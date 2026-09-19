#!/usr/bin/env python3
"""Refresh the copies of the shared firmware sources in the Arduino sketch.

    python tools/gen_arduino_sketch.py           # refresh
    python tools/gen_arduino_sketch.py --check   # verify without writing

Arduino IDE compiles every source file that sits BESIDE the .ino, and it will
not reach outside the sketch folder.  ESP-IDF wants a components tree.  The
two build systems cannot share a directory, so the shared sources are copied.

Copying invites drift, which is the thing this file exists to prevent:
firmware/main/ is the single source of truth, the copies carry a banner saying
so, and tests/test_arduino_sketch.py fails the moment they diverge.  The same
arrangement as protocol.h and protocol.py, for the same reason - a duplicate
that nothing checks is a duplicate that will differ by next month.

Not copied, because each build has its own:

    main.c        ESP-IDF entry point      <-> framing_rover.ino
    net.c         lwIP sockets             <-> net_arduino.cpp  (WiFiUDP)
    hal_esp32.c   ESP-IDF drivers          <-> hal_arduino.cpp  (Arduino core)
"""

from __future__ import annotations

import argparse
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
SRC = ROOT / "firmware" / "main"
DST = ROOT / "firmware_arduino" / "framing_rover"

SHARED = [
    "protocol.h",          # the packet definitions, mirrored by app/protocol.py
    "app_config.h",        # pin map and board settings
    "config_defaults.h",   # generated from config.json
    "rover_hal.h",         # the line between logic and silicon
    "drive.h", "drive.c",  # F-2
    "tilt.h", "tilt.c",    # F-3
    "safety.h", "safety.c",  # F-4
    "net.h",               # implemented by net_arduino.cpp here
    "tasks.h", "tasks.c",  # F-1, shared by both entry points
]

BANNER = """/* === COPY - DO NOT EDIT ===============================================
 *
 * Generated from firmware/main/{name} by tools/gen_arduino_sketch.py.
 * Edit the original and re-run the generator; an edit here is overwritten
 * and tests/test_arduino_sketch.py will fail in the meantime.
 *
 * The copy exists because Arduino IDE only compiles files beside the .ino.
 * ==================================================================== */

"""


def rendered(name: str) -> str:
    return BANNER.format(name=name) + (SRC / name).read_text()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    DST.mkdir(parents=True, exist_ok=True)
    stale = []

    for name in SHARED:
        want = rendered(name)
        target = DST / name
        if args.check:
            if not target.exists() or target.read_text() != want:
                stale.append(name)
        else:
            target.write_text(want)

    if args.check:
        if stale:
            print("the Arduino sketch is out of date: " + ", ".join(stale),
                  file=sys.stderr)
            print("run: python tools/gen_arduino_sketch.py", file=sys.stderr)
            return 1
        print(f"Arduino sketch is up to date ({len(SHARED)} shared files)")
        return 0

    print(f"refreshed {len(SHARED)} shared files in "
          f"{DST.relative_to(ROOT)}")
    hand_written = sorted(p.name for p in DST.glob("*")
                          if p.name not in SHARED and p.name != "secrets.h")
    print("  hand-written, not touched: " + ", ".join(hand_written))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
