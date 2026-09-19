"""comms.py - UDP to the firmware, telemetry back.

W-5.1: UDP, never TCP, for motor commands.  A retransmitted stale command is
worse than a dropped one.

W-5.4: the laptop keeps sending at its fixed rate even when nothing changes.
The packet stream IS the heartbeat - the firmware's watchdog measures the gap
between packets, so a control loop that "has nothing to say" and goes quiet
looks exactly like a crashed laptop, which is the correct interpretation.

The sender therefore runs on its own thread at a fixed rate and repeats the
last command it was given, rather than being driven by the vision loop.
"""

from __future__ import annotations

import logging
import socket
import threading
import time
from dataclasses import dataclass, field

from app import protocol as P

log = logging.getLogger("comms")


@dataclass
class Link:
    """A live link to the firmware.

    Start it, call ``send()`` whenever the decision layer produces a Command,
    and read ``telemetry`` whenever you like.  The heartbeat keeps going in
    between.
    """

    host: str
    port: int = 3333
    telemetry_port: int = 3334
    rate_hz: float = 30.0
    dry_run: bool = False

    telemetry: P.Telemetry | None = field(default=None, init=False)
    telemetry_t: float = field(default=0.0, init=False)
    sent: int = field(default=0, init=False)
    received: int = field(default=0, init=False)
    bad_packets: int = field(default=0, init=False)

    def __post_init__(self):
        self._seq = 0
        self._cmd = P.Command(seq=0, left=0, right=0, tilt_deg=0.0, enable=False)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._sock: socket.socket | None = None
        self._threads: list[threading.Thread] = []
        self._on_telemetry = None

    # -- lifecycle ------------------------------------------------------
    def start(self, on_telemetry=None):
        self._on_telemetry = on_telemetry
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", self.telemetry_port))
        self._sock.settimeout(0.2)

        for target, name in ((self._send_loop, "udp-send"), (self._recv_loop, "udp-recv")):
            thread = threading.Thread(target=target, name=name, daemon=True)
            thread.start()
            self._threads.append(thread)
        log.info("link up: commands to %s:%d at %.0f Hz%s",
                 self.host, self.port, self.rate_hz, "  [DRY RUN]" if self.dry_run else "")
        return self

    def close(self):
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=1.0)
        if self._sock:
            self._sock.close()

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop_motors()
        time.sleep(0.05)
        self.close()

    # -- commands -------------------------------------------------------
    def send(self, command: dict):
        """Queue a Command dict from the decision layer.  It is transmitted on
        the next heartbeat tick and repeated until replaced."""
        left = command.get("left", 0)
        right = command.get("right", 0)
        enable = bool(command.get("enable", False))

        # LG-4 dry-run mode: the full pipeline runs with motor output forced to
        # zero, printing what it would have sent.  Tilt still moves - it cannot
        # drive the rover into anything.
        if self.dry_run and (left or right):
            log.info("DRY RUN would send left=%+5d right=%+5d tilt=%+6.1f enable=%s",
                     left, right, command.get("tilt_deg", 0.0), enable)
            left = right = 0

        with self._lock:
            self._seq = (self._seq + 1) & 0xFFFF
            self._cmd = P.Command(
                seq=self._seq,
                left=left,
                right=right,
                tilt_deg=float(command.get("tilt_deg", 0.0)),
                enable=enable,
                tilt_release=bool(command.get("tilt_release", False)),
                zero_tilt=bool(command.get("zero_tilt", False)),
            )

    def stop_motors(self):
        """Zero duty, enable off.  Sent immediately as well as on the next
        tick, because this is the path that matters."""
        self.send({"left": 0, "right": 0, "tilt_deg": self._cmd.tilt_deg,
                   "enable": False})
        self._transmit()

    def zero_tilt_at(self, measured_pitch_deg: float):
        """Tell the firmware what it is actually looking at.

        H-4.3: there is no home switch and no homing routine.  The phone's
        gravity vector is the absolute tilt reference, so once the phone
        reports a pitch, the laptop hands it to the firmware and the open-loop
        step count is re-based on it.
        """
        self.send({"left": 0, "right": 0, "tilt_deg": measured_pitch_deg,
                   "enable": False, "zero_tilt": True})

    # -- threads --------------------------------------------------------
    def _transmit(self):
        if self._sock is None:
            return
        with self._lock:
            cmd = self._cmd
        try:
            self._sock.sendto(P.pack_command(cmd), (self.host, self.port))
            self.sent += 1
        except OSError as exc:
            # Never raise out of the heartbeat.  A dead link must look like
            # silence to the firmware, which is exactly what it will do.
            log.debug("send failed: %s", exc)

    def _send_loop(self):
        period = 1.0 / max(1.0, self.rate_hz)
        next_t = time.monotonic()
        while not self._stop.is_set():
            self._transmit()
            next_t += period
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()   # fell behind; do not spiral

    def _recv_loop(self):
        while not self._stop.is_set():
            try:
                data, _addr = self._sock.recvfrom(256)
            except (socket.timeout, TimeoutError):
                continue
            except OSError:
                break
            try:
                tlm = P.unpack_telemetry(data)
            except P.ProtocolError as exc:
                self.bad_packets += 1
                if self.bad_packets in (1, 10, 100):
                    log.warning("bad telemetry packet: %s "
                                "(is firmware/main/protocol.h in sync with app/protocol.py?)", exc)
                continue
            self.telemetry = tlm
            self.telemetry_t = time.monotonic()
            self.received += 1
            if self._on_telemetry:
                self._on_telemetry(tlm)

    # -- diagnostics ----------------------------------------------------
    def health(self) -> dict:
        tlm = self.telemetry
        age = time.monotonic() - self.telemetry_t if self.telemetry_t else None
        return {
            "sent": self.sent,
            "received": self.received,
            "bad_packets": self.bad_packets,
            "telemetry_age_s": age,
            "status": P.status_words(tlm.status) if tlm else [],
            "loop_us": tlm.loop_us if tlm else None,
            "range_mm": tlm.range_mm if tlm else None,
            "vbat_mv": tlm.vbat_mv if tlm else None,
        }
