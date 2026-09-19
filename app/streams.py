"""streams.py - newest-frame-only reader and timestamp alignment.

P-1 and P-2, the two pieces of plumbing that quietly ruin a build if they are
got wrong.

P-1 NEWEST-FRAME-ONLY.  A reader thread receives WebSocket messages
continuously and overwrites a single slot; the control loop reads that slot.
Never a queue.  A growing queue presents as latency that climbs slowly through
a session and silently invalidates every gain tuned after it starts - and it
looks exactly like bad gains, so it costs a day before anyone suspects the
transport.

P-2 TIMESTAMP ALIGNMENT.  Motion packets arrive in ~5 ms; frames arrive
150-250 ms late.  Fusing the newest of each pairs an orientation from the
present with an image from the past, which also looks exactly like bad gains.
Each frame therefore looks up the motion values AT ITS OWN TIMESTAMP from a
ring buffer, corrected by a clock offset measured with a ping-pong exchange at
startup.
"""

from __future__ import annotations

import bisect
import logging
import statistics
import threading
import time
from dataclasses import dataclass, field

log = logging.getLogger("streams")


@dataclass
class Frame:
    """One JPEG from the phone, with both clocks attached."""

    jpeg: bytes
    t_phone: float      # phone clock, milliseconds since its epoch, /1000
    t_local: float      # laptop monotonic clock at receipt
    seq: int


class NewestOnly:
    """A one-slot mailbox.  Writers overwrite; the reader takes.

    This is the whole of P-1.  It is four methods and it is the difference
    between a session that holds 200 ms of latency for an hour and one that
    drifts to two seconds without anyone noticing.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._item = None
        self._event = threading.Event()
        self.dropped = 0
        self.accepted = 0

    def put(self, item):
        with self._lock:
            if self._item is not None:
                # The control loop never read the previous one.  That is normal
                # and correct - it means perception is the bottleneck, and the
                # right thing to do with the stale frame is bin it.
                self.dropped += 1
            self._item = item
            self.accepted += 1
        self._event.set()

    def take(self, timeout: float | None = None):
        """Return the newest item, or None.  Clears the slot."""
        if timeout is not None and not self._event.wait(timeout):
            return None
        with self._lock:
            item, self._item = self._item, None
            self._event.clear()
        return item

    def peek(self):
        with self._lock:
            return self._item

    @property
    def drop_rate(self) -> float:
        total = self.accepted or 1
        return self.dropped / total


class MotionBuffer:
    """A ring of motion samples, queryable at an arbitrary timestamp.

    Samples arrive at 50 Hz and are kept for ``window_s`` (default 2 s), which
    is ten times the worst frame latency - enough that a frame's timestamp is
    always inside the buffer, small enough that the bisect stays trivial.
    """

    def __init__(self, window_s: float = 2.0):
        self.window_s = window_s
        self._lock = threading.Lock()
        self._t: list[float] = []
        self._samples: list[dict] = []
        self.count = 0

    def add(self, t_phone: float, sample: dict):
        with self._lock:
            # Out-of-order arrivals are rare but real on a busy Wi-Fi channel;
            # insert in order rather than assuming.
            idx = bisect.bisect_left(self._t, t_phone)
            self._t.insert(idx, t_phone)
            self._samples.insert(idx, sample)
            self.count += 1

            cutoff = self._t[-1] - self.window_s
            drop = bisect.bisect_left(self._t, cutoff)
            if drop > 0:
                del self._t[:drop]
                del self._samples[:drop]

    def at(self, t_phone: float) -> dict | None:
        """Linearly interpolate the motion state at a phone timestamp.

        Returns None when the buffer has nothing near that time, rather than
        the nearest sample - P-7 again: no guesses dressed as measurements.
        """
        with self._lock:
            if not self._t:
                return None
            if t_phone <= self._t[0]:
                return dict(self._samples[0]) if self._t[0] - t_phone < 0.1 else None
            if t_phone >= self._t[-1]:
                return dict(self._samples[-1]) if t_phone - self._t[-1] < 0.1 else None

            idx = bisect.bisect_left(self._t, t_phone)
            t0, t1 = self._t[idx - 1], self._t[idx]
            s0, s1 = self._samples[idx - 1], self._samples[idx]

        span = t1 - t0
        alpha = 0.0 if span <= 0 else (t_phone - t0) / span
        out = {}
        for key in set(s0) | set(s1):
            a, b = s0.get(key), s1.get(key)
            if isinstance(a, (int, float)) and isinstance(b, (int, float)):
                out[key] = a + (b - a) * alpha
            else:
                out[key] = b if alpha > 0.5 else a
        return out

    def latest(self) -> dict | None:
        with self._lock:
            return dict(self._samples[-1]) if self._samples else None


@dataclass
class ClockOffset:
    """P-2's clock offset, measured by a ping-pong exchange at startup.

    The phone stamps frames with its own clock; the laptop needs those in its
    own terms to line them up with anything else.  The exchange is the usual
    NTP-style four-timestamp estimate, and the MEDIAN of several rounds is
    kept - a single round on Wi-Fi can be tens of milliseconds out, and the
    median of nine is stable to a few.
    """

    offset: float = 0.0          # add to a phone timestamp to get laptop time
    rtt: float = 0.0
    samples: list = field(default_factory=list)
    locked: bool = False

    def add_round(self, t_send_local: float, t_phone: float, t_recv_local: float):
        rtt = t_recv_local - t_send_local
        # Assume symmetric delay: the phone's stamp was taken rtt/2 after send.
        offset = (t_send_local + rtt / 2.0) - t_phone
        self.samples.append((offset, rtt))
        if len(self.samples) >= 5:
            self.offset = statistics.median(o for o, _ in self.samples)
            self.rtt = statistics.median(r for _, r in self.samples)
            self.locked = True
        return offset

    def to_local(self, t_phone: float) -> float:
        return t_phone + self.offset

    def to_phone(self, t_local: float) -> float:
        return t_local - self.offset


class Streams:
    """Everything arriving from the phone, in one place.

    The control loop calls ``latest_aligned()`` and gets a frame paired with
    the motion state from that frame's own moment - never the newest motion.
    """

    def __init__(self, motion_window_s: float = 2.0):
        self.frames = NewestOnly()
        self.motion = MotionBuffer(motion_window_s)
        self.clock = ClockOffset()
        self.transcripts = NewestOnly()
        self.hello: dict = {}
        self._latency_samples: list[float] = []

    # -- writers (called by the server's reader task) -------------------
    def push_frame(self, jpeg: bytes, t_phone: float, seq: int = 0):
        now = time.monotonic()
        self.frames.put(Frame(jpeg=jpeg, t_phone=t_phone, t_local=now, seq=seq))
        if self.clock.locked:
            latency = now - self.clock.to_local(t_phone)
            self._latency_samples.append(latency)
            if len(self._latency_samples) > 120:
                del self._latency_samples[:-120]

    def push_motion(self, sample: dict):
        t = sample.get("t")
        if t is None:
            return
        self.motion.add(float(t), sample)

    def push_transcript(self, text: str):
        self.transcripts.put(text)

    # -- reader (called by the control loop) ----------------------------
    def latest_aligned(self, timeout: float | None = None):
        """Return ``(frame, motion_at_that_frame)`` or ``(None, None)``.

        This is P-2 in three lines, and the reason the yaw loop is stable.
        """
        frame = self.frames.take(timeout)
        if frame is None:
            return (None, None)
        motion = self.motion.at(frame.t_phone)
        if motion is None:
            # Better a frame with no orientation than a frame with someone
            # else's orientation.  decide() copes with a missing gyro; it
            # cannot cope with a wrong one.
            motion = {}
        return (frame, motion)

    @property
    def latency_ms(self) -> float | None:
        """Median frame latency over the last ~10 s.

        M4's acceptance test is "latency under 250 ms and flat over 5 min".
        Flat is the important word: a slow climb is the queue that P-1 exists
        to prevent, and this number is how you see it.
        """
        if not self._latency_samples:
            return None
        return statistics.median(self._latency_samples) * 1000.0

    def health(self) -> dict:
        return {
            "latency_ms": self.latency_ms,
            "frames": self.frames.accepted,
            "frames_dropped": self.frames.dropped,
            "drop_rate": round(self.frames.drop_rate, 3),
            "motion_samples": self.motion.count,
            "clock_offset": round(self.clock.offset, 4) if self.clock.locked else None,
            "clock_rtt_ms": round(self.clock.rtt * 1000, 1) if self.clock.locked else None,
        }
