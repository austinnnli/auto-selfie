"""run.py - the control application's main loop.

Not in the PRD's file list; ``session.py`` is the state machine and something
has to be ``__main__``.  This file is deliberately thin - it owns no policy.
It moves data between the modules that do:

    phone --(streams)--> perceive --(Observation)--> session --> decide
                                                                   |
                                    firmware <--(comms)-- Command <-+
                                    phone    <--(say/capture)-------+
                                    disk     <--(record)------------+

    python -m app.run                    # the real thing
    python -m app.run --dry-run          # LG-4: motors forced to zero
    python -m app.run --no-rover         # perception and decisions only
"""

from __future__ import annotations

import argparse
import logging
import os
import pathlib
import signal
import sys
import time

from app import comms as comms_lib
from app import config as config_lib
from app import decide as decide_lib
from app import goal as goal_lib
from app import goal_tier2 as tier2_lib
from app import perceive as perceive_lib
from app import record as record_lib
from app import server as server_lib
from app import session as session_lib
from app.streams import Streams

ROOT = pathlib.Path(__file__).resolve().parents[1]
log = logging.getLogger("run")


class Application:
    def __init__(self, args):
        self.args = args
        self.config = config_lib.load(args.config or config_lib.DEFAULT_PATH)
        self.config.start_watch()

        self.streams = Streams()
        self.server = server_lib.PhoneServer(
            self.streams, self.config.data,
            port=self.config.get("net", {}).get("server_port", 8443))
        self.perceiver = perceive_lib.Perceiver(self.config.data)
        self.recorder = record_lib.Recorder(self.config.data, self.config.hash)
        self.tier2 = tier2_lib.Tier2Parser(
            self.config.data, api_key=os.environ.get("ANTHROPIC_API_KEY"))

        self.link = None
        if not args.no_rover:
            net = self.config.get("net", {})
            self.link = comms_lib.Link(
                host=args.host or net.get("firmware_host", "192.168.4.1"),
                port=net.get("firmware_port", 3333),
                telemetry_port=net.get("telemetry_port", 3334),
                rate_hz=net.get("command_hz", 30),
                dry_run=args.dry_run)

        self.state = session_lib.initial_state(time.monotonic())
        self.running = True
        self.tilt_zeroed = False
        self.last_objects = []
        self._last_obs = None
        self._last_frame = None
        self.pending_transcript = None
        self.stats = {"frames": 0, "decisions": 0, "t0": time.monotonic()}

    # ------------------------------------------------------------------
    def start(self):
        self.server.on_capture = self._on_capture
        self.server.start()
        if self.link:
            self.link.start(on_telemetry=self.recorder.telemetry)
        signal.signal(signal.SIGINT, lambda *_: self.stop())
        log.info("waiting for the phone; open the URL above and press Start")

    def stop(self, *_):
        self.running = False

    def shutdown(self):
        # The stop path runs first and unconditionally.  Everything else here
        # is tidying; this is the part that matters.
        if self.link:
            self.link.stop_motors()
            time.sleep(0.1)
            self.link.close()
        self.server.stop_client()
        self.server.stop()
        self.recorder.close()
        self.config.stop_watch()

        elapsed = time.monotonic() - self.stats["t0"]
        log.info("ran %.0f s: %d frames, %d decisions, %.1f fps, %s",
                 elapsed, self.stats["frames"], self.stats["decisions"],
                 self.stats["frames"] / max(1e-9, elapsed), self.streams.health())

    # ------------------------------------------------------------------
    def _on_capture(self, jpeg: bytes):
        """The phone has sent a full-resolution grab back (C-5).

        D-1's fourth axis is applied here: the scale error is fixed by cropping
        at capture rather than by another manoeuvre, because the last 10% of
        framing is not worth driving for.  The crop is recorded rather than
        applied destructively - the original is what was actually shot.
        """
        meta = {"phase": self.state.get("phase")}
        if self._last_obs is not None:
            crop = decide_lib.capture_crop(self._last_obs, self.state, self.config.data)
            if crop:
                meta["crop"] = [round(v, 4) for v in crop]
        self.recorder.capture(jpeg, meta)

    def _signals(self, now: float) -> dict:
        """Everything arriving from outside the vision loop, in the shape
        session.step() expects."""
        sig = {}
        transcript = self.streams.transcripts.take()
        if transcript == "__tap__":
            sig["tap"] = True
        elif transcript:
            if goal_lib.is_cancel(transcript):
                sig["cancel"] = True
            elif self.state.get("phase") == session_lib.CONFIRM and \
                    goal_lib.is_objection(transcript):
                sig["objection"] = True
            else:
                sig["transcript"] = transcript

        if self.state.get("phase") == session_lib.PARSE:
            sig.update(self._parse_signals(now))
        return sig

    def _parse_signals(self, now: float) -> dict:
        """G-5's two tiers plus G-6's object resolution, as one step."""
        transcript = self.state.get("transcript")
        if not transcript:
            return {"parse_failed": True}

        if self.pending_transcript != transcript:
            self.pending_transcript = transcript
            spec, tier, confidence = tier2_lib.parse(transcript, self.config.data, self.tier2)
            self._tier1 = (spec, confidence)
            threshold = self.config.get("goal", {}).get("parse_confidence", 0.75)
            if confidence >= threshold:
                return self._resolve(spec, 1)
            return {}   # tier 2 is thinking; poll below on the next tick

        answer = self.tier2.poll()
        if answer is not None:
            spec, tier = answer
            return self._resolve(spec, tier)
        if self.tier2.failed:
            # G-5.2: a run never blocks on the network.  Tier 1's answer
            # stands, even at low confidence, and G-8 handles it from there.
            spec, _conf = self._tier1
            return self._resolve(spec, 1)
        return {}

    def _resolve(self, spec: dict, tier: int) -> dict:
        """G-6: resolve the noun phrase against current detections BEFORE
        confirmation, so the confirmation is about a thing the rover can
        actually see."""
        candidates = self.perceiver.resolve_object(
            spec.get("object_phrase", ""), self.last_objects)
        if not candidates:
            self.recorder.goal(spec, tier)
            # G-8: no usable object -> stay in Idle and say why.
            return {"goal_spec": None, "parse_failed": True, "fallback_goal": None}

        chosen = candidates[0]
        spec = dict(spec)
        spec["object_ref"] = chosen
        spec = goal_lib.assign_sides(spec, chosen["cx"])
        self.perceiver.select_object(self._last_frame, (chosen["cx"], chosen["cy"],
                                                        chosen["w"], chosen["h"]),
                                     chosen.get("label", ""))
        self.recorder.goal(spec, tier)
        return {"goal_spec": spec}

    # ------------------------------------------------------------------
    def loop(self):
        cfg = self.config.data
        loop_hz = cfg.get("control", {}).get("loop_hz", 25.0)
        period = 1.0 / max(1.0, loop_hz)
        self._last_frame = None
        next_t = time.monotonic()

        while self.running:
            cfg = self.config.data          # picked up live (CF-1 hot reload)
            now = time.monotonic()

            frame, motion = self.streams.latest_aligned(timeout=period)
            if frame is not None:
                self._last_frame = perceive_lib.decode_jpeg(frame.jpeg)
                self.recorder.frame(frame.jpeg, frame.t_phone, frame.t_local)
                self.stats["frames"] += 1

                obs = self.perceiver.observe(
                    self._last_frame, now,
                    self.streams.clock.to_local(frame.t_phone), motion)
                obs["t_frame_local"] = self.streams.clock.to_local(frame.t_phone)
                if self.link and self.link.telemetry:
                    tlm = self.link.telemetry
                    obs["telemetry"] = {"range_mm": tlm.range_mm if tlm.range_valid else None,
                                        "vbat_mv": tlm.vbat_mv, "status": tlm.status}
                # Read the detections the Observation was built from rather
                # than running the detector again - a second pass would cost
                # another 60-100 ms and halve the frame rate.
                self.last_objects = self.perceiver.last_objects
                self._last_obs = obs
                self.recorder.observation(obs)
            else:
                obs = perceive_lib.blank_observation(now)
                if self.perceiver.last_seen:
                    obs["lost_for"] = now - self.perceiver.last_seen

            # D-9 / H-4.3: once the phone has reported a pitch, hand it to the
            # firmware so its open-loop step count is referenced to gravity.
            if self.link and not self.tilt_zeroed and self.streams.motion.latest():
                pitch = self.streams.motion.latest().get("pitch")
                if pitch is not None:
                    self.link.zero_tilt_at(pitch)
                    self.tilt_zeroed = True
                    log.info("tilt zeroed at the phone's measured pitch %.1f deg", pitch)

            before = self.state.get("phase")
            self.state, events = session_lib.step(
                self.state, obs, cfg, now, self._signals(now))
            if self.state.get("phase") != before:
                why = (self.state.get("transitions") or [{}])[-1].get("why", "")
                log.info("%s -> %s (%s)", before, self.state.get("phase"), why)
                self.recorder.transition(before, self.state.get("phase"), why)

            cmd, self.state = decide_lib.step(obs, self.state, cfg)
            self.stats["decisions"] += 1

            if self.link:
                self.link.send(cmd)
            self.recorder.command(cmd, self.state.get("phase"))

            for event in events:
                if event["kind"] == "say":
                    self.server.say(event["text"])
                    self.recorder.said(event["text"])
            if cmd.get("say"):
                self.server.say(cmd["say"])
                self.recorder.said(cmd["say"])
            if cmd.get("capture"):
                self.server.capture()

            next_t += period
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--config", default=None)
    parser.add_argument("--host", default=None, help="the rover's IP")
    parser.add_argument("--dry-run", action="store_true",
                        help="LG-4: run the full pipeline with motor output forced to zero")
    parser.add_argument("--no-rover", action="store_true",
                        help="perception and decisions only; never opens the UDP link")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-5s %(name)-9s %(message)s",
        datefmt="%H:%M:%S")

    app = Application(args)
    app.start()
    try:
        app.loop()
    except KeyboardInterrupt:
        pass
    finally:
        app.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
