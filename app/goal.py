"""goal.py - PURE.  Tier-1 spoken-goal parser (D3b).

Imports ``math`` and nothing else.

G-5.1: a keyword and preposition grammar covering the G-3 table.
Deterministic, offline, instant.  It must handle every phrase in G-3 without
network access, because a run never blocks on the network.

This module is not in the PRD's file listing - D3b describes the behaviour but
gives it no home.  It lives here rather than in ``session.py`` because it is
pure, table-driven and heavily tested, and because the tier-2 fallback
(``goal_tier2.py``, which does touch the network) needs something to validate
its output against.

    GoalSpec = {
      "raw":           "make me point at the bird",
      "object_phrase": "the bird",
      "object_ref":    <detection id, once resolved>,
      "kind":          "framing" | "pose",
      "pose_goal":     "point_at" | "hold" | "look_at" | None,
      "limb":          "right_arm" | "left_arm" | None,
      "composition":   {subject_x, object_x, eyeline_y,
                        subject_height, size_ratio}
    }

Built once per run and read-only thereafter.
"""

import math

# G-3.  Order matters: the longest and most specific phrases are tested first,
# so "looking at" is not shadowed by "look".
POSE_TRIGGERS = [
    ("pointing at", "point_at"),
    ("point at", "point_at"),
    ("pointing to", "point_at"),
    ("point to", "point_at"),
    ("pointing toward", "point_at"),
    ("point toward", "point_at"),
    ("holding onto", "hold"),
    ("holding", "hold"),
    ("hold onto", "hold"),
    ("hold", "hold"),
    ("touching", "hold"),
    ("touch", "hold"),
    ("pinching", "hold"),
    ("pinch", "hold"),
    ("looking at", "look_at"),
    ("look at", "look_at"),
    ("looking toward", "look_at"),
    ("look toward", "look_at"),
    ("facing", "look_at"),
    ("face", "look_at"),
]

FRAMING_TRIGGERS = [
    "next to", "beside", "in front of", "behind me", "behind",
    "alongside", "by the", "with",
]

# Whole-body template goals.  Not in the G-3 table, but PG-1 lists them and
# they cost one line each here.
TEMPLATE_TRIGGERS = [
    ("arms out", "arms_out"),
    ("arms wide", "arms_out"),
    ("arms down", "arms_down"),
    ("hands on my hips", "hands_hips"),
    ("hands on hips", "hands_hips"),
]

# Words that carry no meaning for us but are always in the sentence.
LEAD_FILLER = (
    "hey rover", "ok rover", "okay rover", "rover",
    "please", "can you", "could you", "i want you to", "i'd like you to",
    "take a photo of me", "take a picture of me", "take a photo", "take a picture",
    "get a shot of me", "get a photo of me", "shoot me", "make me", "make it",
    "photograph me", "frame me", "put me", "have me", "get me", "let me",
    "i want to be", "i want to look like i'm", "i want to look like im",
)

TRAIL_FILLER = ("please", "thanks", "thank you", "ok", "okay")

LIMB_PHRASES = [
    ("my left hand", "left_arm"), ("my left arm", "left_arm"),
    ("your left hand", "left_arm"), ("your left arm", "left_arm"),
    ("the left hand", "left_arm"), ("left hand", "left_arm"), ("left arm", "left_arm"),
    ("my right hand", "right_arm"), ("my right arm", "right_arm"),
    ("your right hand", "right_arm"), ("your right arm", "right_arm"),
    ("the right hand", "right_arm"), ("right hand", "right_arm"), ("right arm", "right_arm"),
]

# G-3: "The user only ever states a default to override it."
COMPOSITION_OVERRIDES = [
    (("on the left", "to the left", "left third", "left side"), {"subject_x": 0.333}),
    (("on the right", "to the right", "right third", "right side"), {"subject_x": 0.667}),
    (("in the middle", "centred", "centered", "in the centre", "in the center", "dead centre"),
     {"subject_x": 0.5, "object_x": 0.5}),
    (("full body", "head to toe", "whole body", "full length"), {"subject_height": 0.85}),
    (("head and shoulders", "close up", "close-up", "portrait"), {"subject_height": 0.35}),
    (("wide shot", "wide", "far back", "zoomed out"), {"subject_height": 0.3}),
    (("low angle", "from below"), {"eyeline_y": 0.6}),
    (("high angle", "from above"), {"eyeline_y": 0.2}),
]

CANCEL_WORDS = ("cancel", "stop", "never mind", "nevermind", "forget it", "abort", "wait no")
OBJECTION_WORDS = ("no", "nope", "wrong", "not that", "that's wrong", "thats wrong", "stop")

VALID_KINDS = ("framing", "pose")
VALID_POSE_GOALS = ("point_at", "hold", "look_at", "beside",
                    "arms_out", "arms_down", "hands_hips", "no_overlap")
VALID_LIMBS = ("left_arm", "right_arm", None)


def normalise(text):
    """Lower-case, strip punctuation, collapse whitespace.  Speech recognition
    output is already unpunctuated most of the time, but not always."""
    if not text:
        return ""
    out = []
    for ch in text.lower():
        if ch.isalnum() or ch == " " or ch == "'":
            out.append(ch)
        else:
            out.append(" ")
    return " ".join("".join(out).split())


def _strip_filler(text):
    changed = True
    while changed:
        changed = False
        for lead in LEAD_FILLER:
            if text.startswith(lead + " "):
                text = text[len(lead) + 1:]
                changed = True
            elif text == lead:
                return ""
        for trail in TRAIL_FILLER:
            if text.endswith(" " + trail):
                text = text[: -(len(trail) + 1)]
                changed = True
    return text.strip()


def extract_limb(text):
    """G-4: an explicit limb wins over the object-side heuristic.

    The limb phrase is removed from the text so it cannot be mistaken for the
    object - "point at the sign with my left hand" must not resolve an object
    called "my left hand".
    """
    for phrase, limb in LIMB_PHRASES:
        for prefix in ("with ", "using ", ""):
            needle = prefix + phrase
            idx = text.find(needle)
            if idx >= 0:
                return limb, (text[:idx] + " " + text[idx + len(needle):]).strip()
    return None, text


def extract_composition(text, defaults):
    """Pull any stated overrides out of the sentence.

    Returns ``(composition, remaining_text)``.  Matched phrases are removed so
    they do not end up inside the object phrase.
    """
    comp = dict(defaults)
    for phrases, override in COMPOSITION_OVERRIDES:
        for phrase in phrases:
            idx = text.find(phrase)
            if idx >= 0:
                comp.update(override)
                text = (text[:idx] + " " + text[idx + len(phrase):]).strip()
                text = " ".join(text.split())
                break
    return comp, text


def _clean_object_phrase(phrase):
    phrase = " ".join(phrase.split())
    for trail in TRAIL_FILLER + ("for me", "in the shot", "in shot", "in frame"):
        if phrase.endswith(" " + trail):
            phrase = phrase[: -(len(trail) + 1)]
    # A trailing conjunction is a sign the sentence was cut off.
    for trail in (" and", " with", " or"):
        if phrase.endswith(trail):
            phrase = phrase[: -len(trail)]
    return phrase.strip()


def classify(text):
    """The verb decides ``kind``; everything else falls through to plain
    framing (G-3).  Returns ``(kind, pose_goal, object_phrase, confidence)``."""
    for phrase, goal_id in TEMPLATE_TRIGGERS:
        idx = text.find(phrase)
        if idx >= 0:
            rest = _clean_object_phrase(text[:idx] + " " + text[idx + len(phrase):])
            return ("pose", goal_id, rest, 0.9)

    for phrase, goal_id in POSE_TRIGGERS:
        idx = text.find(phrase)
        if idx >= 0:
            obj = _clean_object_phrase(text[idx + len(phrase):])
            if obj:
                return ("pose", goal_id, obj, 0.95)
            # "point at" with nothing after it is a half-heard sentence.
            return ("pose", goal_id, "", 0.4)

    for phrase in FRAMING_TRIGGERS:
        idx = text.find(phrase)
        if idx >= 0:
            obj = _clean_object_phrase(text[idx + len(phrase):])
            if obj:
                return ("framing", None, obj, 0.9)
            return ("framing", None, "", 0.4)

    # A bare object with no verb is a framing run (G-3, last row).
    obj = _clean_object_phrase(text)
    if obj:
        # Lower confidence: a bare noun phrase is also what a misheard sentence
        # looks like, so this is the band where tier 2 earns its keep.
        return ("framing", None, obj, 0.78)
    return ("framing", None, "", 0.0)


def is_cancel(text):
    t = normalise(text)
    return any(t == w or t.startswith(w + " ") for w in CANCEL_WORDS)


def is_objection(text):
    t = normalise(text)
    if not t:
        return False
    return any(t == w or t.startswith(w + " ") for w in OBJECTION_WORDS)


def parse(raw, config):
    """Tier 1.  Returns ``(goal_spec, confidence)``.

    ``goal_spec`` always has the full shape, even at low confidence, so the
    caller can fall back to it after tier 2 fails (G-8's second rung).
    """
    defaults = dict(config.get("composition_defaults", {}))
    text = _strip_filler(normalise(raw))
    limb, text = extract_limb(text)
    comp, text = extract_composition(text, defaults)
    kind, pose_goal, object_phrase, confidence = classify(text)

    spec = {
        "raw": raw,
        "object_phrase": object_phrase,
        "object_ref": None,
        "kind": kind,
        "pose_goal": pose_goal,
        "limb": limb,
        "composition": comp,
        "tier": 1,
        "locked": False,
    }
    if not object_phrase:
        confidence = min(confidence, 0.35)
    return spec, confidence


def assign_sides(spec, object_cx):
    """G-4: for a framing run, the object's current position in frame decides
    which third the subject takes - subject opposite object.

    Called once, after the object is resolved against a real detection and
    before the spec is locked.
    """
    out = dict(spec)
    comp = dict(out.get("composition") or {})
    if object_cx is None:
        return out

    sx = comp.get("subject_x", 0.333)
    ox = comp.get("object_x", 0.667)
    thirds = (min(sx, ox), max(sx, ox))

    if object_cx >= 0.5:
        comp["object_x"], comp["subject_x"] = thirds[1], thirds[0]
    else:
        comp["object_x"], comp["subject_x"] = thirds[0], thirds[1]
    out["composition"] = comp
    return out


def validate(spec):
    """G-5.2: tier-2 output is validated against the schema and rejected if it
    names a goal that does not exist.

    Returns ``(ok, reason)``.  Used on the language model's JSON, and on
    anything else that claims to be a GoalSpec.
    """
    if not isinstance(spec, dict):
        return (False, "not an object")
    if spec.get("kind") not in VALID_KINDS:
        return (False, f"kind {spec.get('kind')!r} is not one of {VALID_KINDS}")
    if spec.get("kind") == "pose":
        if spec.get("pose_goal") not in VALID_POSE_GOALS:
            return (False, f"pose_goal {spec.get('pose_goal')!r} does not exist")
    elif spec.get("pose_goal") not in (None, ""):
        return (False, "a framing run must not carry a pose_goal")
    if spec.get("limb") not in VALID_LIMBS:
        return (False, f"limb {spec.get('limb')!r} is not a limb")
    comp = spec.get("composition")
    if comp is not None:
        if not isinstance(comp, dict):
            return (False, "composition is not an object")
        for key, value in comp.items():
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                return (False, f"composition.{key} is not a number")
            if not (0.0 <= float(value) <= 10.0):
                return (False, f"composition.{key} is out of range")
    if not isinstance(spec.get("object_phrase", ""), str):
        return (False, "object_phrase is not a string")
    return (True, "")


def confirmation_line(spec):
    """G-7: the rover speaks the parsed goal back in its own words, then waits
    ``confirm_window`` for an objection.  A wrong parse must cost two seconds,
    not a whole run.

    Deliberately paraphrased rather than echoed - reading the sentence back
    verbatim proves only that the microphone works, not that the parse is right.
    """
    obj = spec.get("object_phrase") or "what I can see"
    comp = spec.get("composition") or {}
    sx = comp.get("subject_x", 0.333)
    side = "left third" if sx < 0.45 else ("right third" if sx > 0.55 else "middle")

    if spec.get("kind") == "pose":
        goal_id = spec.get("pose_goal")
        limb = spec.get("limb")
        hand = ""
        if limb:
            hand = f" with your {'left' if limb == 'left_arm' else 'right'} hand"
        verb = {
            "point_at": f"Pointing at {obj}{hand}",
            "hold": f"Holding {obj}{hand}",
            "look_at": f"Looking at {obj}",
            "arms_out": "Arms out",
            "arms_down": "Arms down",
            "hands_hips": "Hands on hips",
            "beside": f"Beside {obj}",
            "no_overlap": f"Clear of {obj}",
        }.get(goal_id, f"{goal_id} {obj}")
        return f"{verb}, you on the {side}. Starting."

    return f"Framing you with {obj}, you on the {side}. Starting."
