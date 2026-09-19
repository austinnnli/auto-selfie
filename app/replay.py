"""replay.py - re-run a log through the same perceive and decide code.

LG-2: feeds a recorded log back through the same ``perceive`` and ``decide``
code at up to 10x speed with no hardware attached.

    python -m app.replay logs/20260918-143210
    python -m app.replay logs/latest --speed 10 --from-frames
    python -m app.replay logs/latest --config config.tuned.json --diff

Built during the video-pipeline milestone, not at the end.  Built early it
accelerates every later milestone; built late it is never built at all.

Two modes:

  --from-obs (default)  replay the recorded Observations through decide().
                        Instant, deterministic, and the right tool for tuning
                        a gain: perception is held fixed so any change in the
                        output came from the change you made.

  --from-frames         re-run the saved JPEGs through perceive() as well.
                        Slower, and the right tool when the detector or the
                        tracker changed.

``--diff`` replays against the recorded commands and prints where the new code
disagrees with what actually happened.  That is the everyday workflow: change a
gain, see exactly which frames now behave differently.
"""

from __future__ import annotations

import argparse
import json
import logging
import pathlib
import sys
import time

from app import decide as decide_lib
from app import record as record_lib
from app import session as session_lib

log = logging.getLogger("replay")


def load_config(log_dir: pathlib.Path, override: str | None) -> tuple[dict, str]:
    """The config that produced the run, unless one is given explicitly.

    Replaying with today's config by accident is how you spend an afternoon
    chasing a difference you introduced yourself.
    """
    if override:
        return json.loads(pathlib.Path(override).read_text()), f"override:{override}"
    for record in record_lib.read_log(log_dir):
        if record.get("kind") == "meta":
            return record.get("config", {}), record.get("config_hash", "")
    raise SystemExit(f"{log_dir} has no meta record - was it written by record.py?")


def replay(log_dir, config=None, speed=10.0, from_frames=False, diff=False,
           limit=None, verbose=False):
    """Returns ``(stats, rows)``.  ``rows`` is one dict per decision."""
    log_dir = pathlib.Path(log_dir)
    cfg, cfg_hash = load_config(log_dir, config)

    perceiver = None
    if from_frames:
        from app import perceive as perceive_lib
        perceiver = perceive_lib.Perceiver(cfg)

    state = session_lib.initial_state(0.0)
    rows = []
    recorded_cmds = []
    mismatches = 0
    wall0 = time.monotonic()
    log_t0 = None
    pending_frame = None

    for record in record_lib.read_log(log_dir):
        kind = record.get("kind")

        if kind == "goal":
            spec = dict(record.get("spec") or {})
            spec["locked"] = True
            state["goal"] = spec
            if state.get("phase") in (session_lib.IDLE, session_lib.LISTEN):
                state["phase"] = session_lib.FRAME
            continue

        if kind == "phase":
            state["phase"] = record.get("to", state.get("phase"))
            continue

        if kind == "frame":
            pending_frame = record
            continue

        if kind == "cmd":
            recorded_cmds.append(record.get("cmd"))
            continue

        if kind != "obs":
            continue

        obs = record["obs"]
        if from_frames and pending_frame is not None:
            from app import perceive as perceive_lib
            jpeg_path = log_dir / "frames" / pending_frame["file"]
            if jpeg_path.exists():
                frame = perceive_lib.decode_jpeg(jpeg_path.read_bytes())
                if frame is not None:
                    obs = perceiver.observe(frame, obs.get("t", 0.0),
                                            obs.get("t_frame", 0.0),
                                            obs.get("gyro", {}))
            pending_frame = None

        # 10x means 10x, not "as fast as possible": the point is to watch a run
        # back, and an unthrottled replay of a two-minute run is a blur.
        if log_t0 is None:
            log_t0 = record["t"]
        if speed > 0:
            target = (record["t"] - log_t0) / speed
            behind = target - (time.monotonic() - wall0)
            if behind > 0:
                time.sleep(behind)

        cmd, state = decide_lib.step(obs, state, cfg)
        row = {"t": record["t"], "phase": state.get("phase"), "cmd": cmd}
        rows.append(row)

        if verbose:
            print(f"  t={record['t']:8.3f} {state.get('phase'):8s} "
                  f"L{cmd['left']:+5d} R{cmd['right']:+5d} tilt{cmd['tilt_deg']:+6.1f}"
                  + (f"  say={cmd['say']!r}" if cmd["say"] else ""))

        if limit and len(rows) >= limit:
            break

    # The diff runs AFTER the loop, not inside it.  In the log a command is
    # written after the observation that produced it, so comparing as we go
    # would always be reaching for a record that has not been read yet - and
    # would silently compare nothing at all, which is worse than being wrong.
    if diff:
        for row, was in zip(rows, recorded_cmds):
            if was and _differs(was, row["cmd"]):
                mismatches += 1
                cmd = row["cmd"]
                print(f"  t={row['t']:8.3f}  was L{was['left']:+5d} R{was['right']:+5d} "
                      f"tilt{was['tilt_deg']:+6.1f}   now L{cmd['left']:+5d} "
                      f"R{cmd['right']:+5d} tilt{cmd['tilt_deg']:+6.1f}")
        if len(rows) != len(recorded_cmds):
            print(f"  note: {len(rows)} decisions replayed against "
                  f"{len(recorded_cmds)} recorded commands")

    stats = {
        "log": str(log_dir),
        "config_hash": cfg_hash,
        "decisions": len(rows),
        "recorded_commands": len(recorded_cmds),
        "mismatches": mismatches,
        "wall_s": round(time.monotonic() - wall0, 2),
        "phases": sorted({r["phase"] for r in rows}),
    }
    return stats, rows


def _differs(a: dict, b: dict) -> bool:
    return (a.get("left") != b.get("left")
            or a.get("right") != b.get("right")
            or abs(float(a.get("tilt_deg", 0)) - float(b.get("tilt_deg", 0))) > 0.05
            or bool(a.get("enable")) != bool(b.get("enable"))
            or bool(a.get("capture")) != bool(b.get("capture")))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("log", help="a run directory under logs/")
    parser.add_argument("--speed", type=float, default=10.0,
                        help="replay speed; 0 means as fast as possible")
    parser.add_argument("--from-frames", action="store_true",
                        help="re-run perception over the saved JPEGs too")
    parser.add_argument("--config", default=None,
                        help="replay with a different config.json")
    parser.add_argument("--diff", action="store_true",
                        help="print where the new code disagrees with the run")
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    stats, _rows = replay(args.log, args.config, args.speed, args.from_frames,
                          args.diff, args.limit, args.verbose)

    print()
    for key, value in stats.items():
        print(f"  {key:20s} {value}")

    if args.diff:
        # M7's acceptance test: a logged run re-runs at 10x with identical
        # output.  Any mismatch is a code bug found at a desk instead of a
        # mystery found in a field (LG-3's argument, applied to whole runs).
        if stats["mismatches"]:
            print(f"\n{stats['mismatches']} of {stats['decisions']} decisions differ")
            return 1
        print("\nidentical output - the replay reproduces the run exactly")
    return 0


if __name__ == "__main__":
    sys.exit(main())
