"""teleop.py - keyboard driving and the safety drills (M8).

    python -m app.teleop                      # live, wheels off the ground first
    python -m app.teleop --dry-run            # prints what it would have sent
    python -m app.teleop --drill watchdog     # run one of the hardware drills

Working rule 1: wheels off the ground for the first test of any new logic.
Working rule 2: dry-run mode before energising.

This is the tool that proves the whole command path before any control law
runs on top of it, and the one you come back to whenever the firmware changes.
"""

from __future__ import annotations

import argparse
import json
import logging
import pathlib
import sys
import termios
import time
import tty

from app import comms as comms_lib
from app import protocol as P

ROOT = pathlib.Path(__file__).resolve().parents[1]
log = logging.getLogger("teleop")

HELP = """
  w / s    forward / reverse        a / d    turn left / right
  space    stop                     x        zero all
  [ / ]    tilt down / up           r        release tilt coils
  + / -    faster / slower          t        print telemetry
  q        quit (motors stopped on the way out)
"""


def drive_keys(link: comms_lib.Link, config: dict):
    hw = config.get("hardware", {})
    speed = hw.get("min_duty_left", 320) + 60
    max_duty = hw.get("max_duty", 650)
    tilt = 0.0
    left = right = 0

    print(HELP)
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            key = sys.stdin.read(1)
            if key == "q":
                break
            elif key == "w":
                left = right = speed
            elif key == "s":
                left = right = -speed
            elif key == "a":
                left, right = -speed, speed
            elif key == "d":
                left, right = speed, -speed
            elif key == " ":
                left = right = 0
            elif key == "x":
                left = right = 0
                tilt = 0.0
            elif key == "[":
                tilt = max(hw.get("tilt_min_deg", -35.0), tilt - 2.0)
            elif key == "]":
                tilt = min(hw.get("tilt_max_deg", 35.0), tilt + 2.0)
            elif key == "r":
                link.send({"left": 0, "right": 0, "tilt_deg": tilt,
                           "enable": False, "tilt_release": True})
                print("tilt coils released")
                continue
            elif key == "+":
                speed = min(max_duty, speed + 25)
            elif key == "-":
                speed = max(0, speed - 25)
            elif key == "t":
                print("  " + json.dumps(link.health()))
                continue
            else:
                continue

            link.send({"left": left, "right": right, "tilt_deg": tilt, "enable": True})
            print(f"\r  L{left:+5d} R{right:+5d} tilt{tilt:+6.1f} speed{speed:4d}   ",
                  end="", flush=True)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        link.stop_motors()
        print("\nmotors stopped")


def drill_watchdog(link: comms_lib.Link, config: dict):
    """Drill 1: motors must stop in under 250 ms when the packets stop.

    The real drill is disabling the laptop's Wi-Fi mid-drive and checking on
    video (M2).  This is the bench version: stop sending and watch the
    watchdog bit appear in telemetry.
    """
    watchdog_ms = config.get("safety", {}).get("watchdog_ms", 250)
    print(f"driving for 2 s, then going silent.  watchdog is {watchdog_ms} ms.")
    link.send({"left": 400, "right": 400, "tilt_deg": 0.0, "enable": True})
    time.sleep(2.0)

    link._stop.set()          # stop the heartbeat without closing the socket
    t0 = time.monotonic()
    tripped_at = None
    while time.monotonic() - t0 < 2.0:
        tlm = link.telemetry
        if tlm and tlm.watchdog_trip:
            tripped_at = (time.monotonic() - t0) * 1000
            break
        time.sleep(0.005)

    if tripped_at is None:
        print("FAIL  the watchdog never tripped - motors are still live")
        return 1
    margin = watchdog_ms * 1.4
    verdict = "PASS" if tripped_at < margin else "FAIL"
    print(f"{verdict}  watchdog tripped after {tripped_at:.0f} ms "
          f"(limit {watchdog_ms} ms, allowing one telemetry period)")
    return 0 if verdict == "PASS" else 1


def drill_range(link: comms_lib.Link, config: dict):
    """Drill 2: hand over the sensor at full forward.  Forward must be
    refused; reverse must still be allowed."""
    print("commanding full forward.  Put your hand over the range sensor.")
    link.send({"left": 500, "right": 500, "tilt_deg": 0.0, "enable": True})

    deadline = time.monotonic() + 15.0
    saw_trip = False
    while time.monotonic() < deadline:
        tlm = link.telemetry
        if tlm and tlm.range_trip:
            saw_trip = True
            print(f"  range trip at {tlm.range_mm} mm - forward should now be refused")
            break
        time.sleep(0.05)

    if not saw_trip:
        print("FAIL  no range trip seen in 15 s (is the sensor wired and its "
              "driver dropped into hal_range_read_mm?)")
        return 1

    print("  now commanding reverse - the wheels should still turn")
    link.send({"left": -450, "right": -450, "tilt_deg": 0.0, "enable": True})
    time.sleep(2.0)
    link.stop_motors()
    print("PASS  if the wheels reversed while your hand was over the sensor")
    return 0


def drill_backlash(link: comms_lib.Link, config: dict):
    """Drill 3 (M3, CF-2): measure backlash_steps.

    Reverse direction and count steps until motion begins.  The phone's pitch
    is the reference; watch the telemetry's tilt_steps against what the phone
    reports and put the difference in config.json.
    """
    print("sweeping tilt up then down.  Watch the phone's pitch reading:")
    print("the steps commanded before the pitch starts moving ARE backlash_steps.\n")
    for target in (0.0, 20.0, 0.0, -20.0, 0.0):
        link.send({"left": 0, "right": 0, "tilt_deg": target, "enable": True})
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            tlm = link.telemetry
            steps = tlm.tilt_steps if tlm else 0
            print(f"\r  target {target:+6.1f}  steps {steps:+7d}   ", end="", flush=True)
            time.sleep(0.05)
        print()
    link.stop_motors()
    return 0


DRILLS = {"watchdog": drill_watchdog, "range": drill_range, "backlash": drill_backlash}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--host", default=None)
    parser.add_argument("--dry-run", action="store_true",
                        help="LG-4: print what would be sent, force motor output to zero")
    parser.add_argument("--drill", choices=sorted(DRILLS))
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    config = json.loads((ROOT / "config.json").read_text())
    net = config.get("net", {})

    link = comms_lib.Link(
        host=args.host or net.get("firmware_host", "192.168.4.1"),
        port=net.get("firmware_port", 3333),
        telemetry_port=net.get("telemetry_port", 3334),
        rate_hz=net.get("command_hz", 30),
        dry_run=args.dry_run,
    )
    link.start()
    try:
        if args.drill:
            return DRILLS[args.drill](link, config)
        drive_keys(link, config)
        return 0
    finally:
        link.stop_motors()
        time.sleep(0.1)
        link.close()


if __name__ == "__main__":
    sys.exit(main())
