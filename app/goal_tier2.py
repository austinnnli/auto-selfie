"""goal_tier2.py - the cloud fallback parser (G-5.2).

Anything tier 1 cannot classify above ``parse_confidence`` goes to a language
model that returns the same JSON schema.  Its output is validated against the
schema and rejected if it names a goal that does not exist.

A RUN NEVER BLOCKS ON THE NETWORK.  If tier 2 is unreachable, the system asks
once and then falls back to framing (G-5.2, G-8).  That rule is enforced here
rather than trusted to the caller: the request runs on a worker thread with a
hard timeout, and the only way to get an answer out of this module is to ask
whether one has arrived yet.

Impure by definition - it is the one module in the goal path that touches the
network.  ``goal.py`` stays pure so its grammar can be tested and ported.
"""

from __future__ import annotations

import json
import logging
import threading
import urllib.error
import urllib.request

from app import goal as goal_lib

log = logging.getLogger("tier2")

SCHEMA_PROMPT = """You convert a spoken photography instruction into JSON.

Return ONLY a JSON object with exactly these keys:
  object_phrase : string  - the thing in the scene, e.g. "the red door". "" if none.
  kind          : "framing" or "pose"
  pose_goal     : one of "point_at","hold","look_at","beside","arms_out",
                  "arms_down","hands_hips","no_overlap", or null for framing
  limb          : "left_arm", "right_arm", or null
  composition   : object, only keys the speaker explicitly asked for, from
                  subject_x, object_x, eyeline_y, subject_height, size_ratio
                  (all numbers 0..1 except size_ratio). {} if unstated.

Rules:
- A verb of pointing/holding/touching/looking makes it a pose run.
- "next to", "beside", "in front of", "with" and a bare noun make it framing.
- Never invent a pose_goal that is not in the list above.
- Never invent composition values the speaker did not state.

Instruction: %s
JSON:"""


class Tier2Parser:
    """Non-blocking wrapper around a language-model call.

    ``submit()`` starts a request and returns immediately.  ``poll()`` returns
    ``(spec, tier)`` once an answer has arrived and validated, or ``None``.
    ``failed`` goes True on timeout, network error or schema rejection, and the
    caller drops to G-8's failure ladder.
    """

    def __init__(self, config: dict, endpoint: str | None = None, api_key: str | None = None):
        gcfg = config.get("goal", {})
        self.enabled = gcfg.get("tier2_enabled", True)
        self.timeout = gcfg.get("tier2_timeout_s", 5.0)
        self.model = gcfg.get("tier2_model", "claude-sonnet-4-5")
        self.endpoint = endpoint or "https://api.anthropic.com/v1/messages"
        self.api_key = api_key
        self.config = config

        self._lock = threading.Lock()
        self._result = None
        self._thread: threading.Thread | None = None
        self.failed = False
        self.raw_response = ""

    def submit(self, transcript: str):
        if not self.enabled or not self.api_key:
            self.failed = True
            log.info("tier 2 is off or has no API key; tier 1's answer stands")
            return
        with self._lock:
            self._result = None
            self.failed = False
        self._thread = threading.Thread(
            target=self._run, args=(transcript,), name="tier2", daemon=True)
        self._thread.start()

    def poll(self):
        with self._lock:
            return self._result

    def _run(self, transcript: str):
        try:
            text = self._request(transcript)
            self.raw_response = text
            spec = self._to_spec(transcript, text)
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            log.warning("tier 2 unreachable (%s); falling back", exc)
            self.failed = True
            return
        except ValueError as exc:
            log.warning("tier 2 returned something unusable (%s); falling back", exc)
            self.failed = True
            return

        with self._lock:
            self._result = (spec, 2)

    def _request(self, transcript: str) -> str:
        body = json.dumps({
            "model": self.model,
            "max_tokens": 300,
            "messages": [{"role": "user", "content": SCHEMA_PROMPT % transcript}],
        }).encode()
        req = urllib.request.Request(
            self.endpoint, data=body, method="POST",
            headers={"content-type": "application/json",
                     "x-api-key": self.api_key,
                     "anthropic-version": "2023-06-01"})
        with urllib.request.urlopen(req, timeout=self.timeout) as resp:
            payload = json.loads(resp.read().decode())
        blocks = payload.get("content") or []
        return "".join(b.get("text", "") for b in blocks if isinstance(b, dict))

    def _to_spec(self, transcript: str, text: str) -> dict:
        """Parse, then VALIDATE.  G-5.2: rejected if it names a goal that does
        not exist.  A model that invents ``pose_goal: "jump"`` must not be able
        to put the session into a state with no handler."""
        start, end = text.find("{"), text.rfind("}")
        if start < 0 or end <= start:
            raise ValueError("no JSON object in the response")
        data = json.loads(text[start:end + 1])

        comp = dict(self.config.get("composition_defaults", {}))
        stated = data.get("composition") or {}
        if not isinstance(stated, dict):
            raise ValueError("composition is not an object")
        comp.update({k: v for k, v in stated.items() if k in comp})

        spec = {
            "raw": transcript,
            "object_phrase": str(data.get("object_phrase") or ""),
            "object_ref": None,
            "kind": data.get("kind"),
            "pose_goal": data.get("pose_goal"),
            "limb": data.get("limb"),
            "composition": comp,
            "tier": 2,
            "locked": False,
        }
        ok, reason = goal_lib.validate(spec)
        if not ok:
            raise ValueError(reason)
        return spec


def parse(transcript: str, config: dict, tier2: Tier2Parser | None = None):
    """The G-5 two-tier parser as one call.

    Returns ``(spec, tier, confidence)``.  Tier 1 answers instantly and its
    answer stands unless it is below ``parse_confidence``; tier 2 is only ever
    consulted in that band, and never blocks a run.
    """
    spec, confidence = goal_lib.parse(transcript, config)
    threshold = config.get("goal", {}).get("parse_confidence", 0.75)

    if confidence >= threshold or tier2 is None:
        return spec, 1, confidence

    tier2.submit(transcript)
    return spec, 1, confidence
