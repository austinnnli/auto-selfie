"""streams.py: P-1 newest-frame-only and P-2 timestamp alignment.

Both are transport plumbing, and both fail in the same way if they are wrong -
as latency that climbs slowly through a session, which looks exactly like bad
gains and costs a day before anyone suspects the transport.  Hence tests.
"""

import pytest

from app.streams import ClockOffset, Frame, MotionBuffer, NewestOnly, Streams


# ---------------------------------------------------------------------------
# P-1  never a queue
# ---------------------------------------------------------------------------

def test_the_slot_holds_only_the_newest():
    slot = NewestOnly()
    for i in range(100):
        slot.put(i)
    assert slot.take() == 99
    assert slot.take() is None


def test_overwrites_are_counted_as_drops():
    """A high drop rate is normal and correct - it means perception is the
    bottleneck.  It becomes a number on screen rather than hidden latency."""
    slot = NewestOnly()
    for i in range(10):
        slot.put(i)
    slot.take()
    assert slot.dropped == 9
    assert slot.accepted == 10
    assert slot.drop_rate == pytest.approx(0.9)


def test_taking_clears_the_slot():
    slot = NewestOnly()
    slot.put("a")
    assert slot.take() == "a"
    assert slot.peek() is None


def test_a_flood_never_grows_memory():
    """The whole point: no unbounded structure anywhere on the frame path."""
    slot = NewestOnly()
    for i in range(100000):
        slot.put(bytes(1000))
    assert slot.peek() is not None
    slot.take()
    assert slot.peek() is None


# ---------------------------------------------------------------------------
# P-2  alignment
# ---------------------------------------------------------------------------

def test_motion_is_interpolated_at_the_requested_time():
    buf = MotionBuffer()
    for i in range(10):
        buf.add(i * 0.02, {"rate_z": i * 10.0, "pitch": -i * 1.0})
    mid = buf.at(0.05)
    assert mid["rate_z"] == pytest.approx(25.0)
    assert mid["pitch"] == pytest.approx(-2.5)


def test_a_frame_gets_the_orientation_from_its_own_moment():
    """P-2: fusing the newest of each pairs an orientation from the present
    with an image from the past.  A frame stamped 200 ms ago must get the
    orientation from 200 ms ago, not the one that just arrived."""
    streams = Streams()
    streams.clock.offset = 0.0
    streams.clock.locked = True

    for i in range(100):
        streams.push_motion({"t": i * 0.02, "rate_z": float(i)})
    streams.push_frame(b"jpeg", t_phone=0.50)      # 500 ms into the sequence

    frame, motion = streams.latest_aligned()
    assert frame.jpeg == b"jpeg"
    assert motion["rate_z"] == pytest.approx(25.0), \
        "the frame was paired with the newest motion instead of its own"


def test_out_of_order_motion_samples_are_inserted_in_order():
    buf = MotionBuffer()
    for t in (0.00, 0.04, 0.02, 0.06):
        buf.add(t, {"rate_z": t * 100})
    assert buf.at(0.03)["rate_z"] == pytest.approx(3.0)


def test_the_ring_buffer_is_bounded():
    buf = MotionBuffer(window_s=0.5)
    for i in range(10000):
        buf.add(i * 0.02, {"rate_z": 0.0})
    assert len(buf._t) < 60


def test_a_frame_from_outside_the_window_gets_nothing():
    """Better a frame with no orientation than a frame with someone else's."""
    buf = MotionBuffer(window_s=1.0)
    for i in range(10):
        buf.add(10.0 + i * 0.02, {"rate_z": 1.0})
    assert buf.at(0.0) is None
    assert buf.at(1000.0) is None


def test_an_empty_buffer_returns_nothing():
    assert MotionBuffer().at(1.0) is None
    assert MotionBuffer().latest() is None


def test_alignment_survives_missing_motion():
    """decide() copes with a missing gyro; it cannot cope with a wrong one."""
    streams = Streams()
    streams.push_frame(b"jpeg", t_phone=5.0)
    frame, motion = streams.latest_aligned()
    assert frame is not None
    assert motion == {}


# ---------------------------------------------------------------------------
# P-2  the clock offset
# ---------------------------------------------------------------------------

def test_the_offset_is_the_median_of_several_rounds():
    """A single round on Wi-Fi can be tens of milliseconds out; the median of
    nine is stable to a few."""
    clock = ClockOffset()
    # True offset 10.0 s, symmetric 200 ms rtt, with one wild outlier.
    for i in range(4):
        clock.add_round(100.0 + i, 90.0 + i, 100.2 + i)
    clock.add_round(200.0, 188.0, 203.0)          # outlier
    for i in range(4):
        clock.add_round(300.0 + i, 290.0 + i, 300.2 + i)
    assert clock.locked
    assert clock.offset == pytest.approx(10.1, abs=0.01)


def test_the_offset_is_not_used_before_it_locks():
    clock = ClockOffset()
    clock.add_round(100.0, 90.0, 100.2)
    assert not clock.locked
    assert clock.offset == 0.0


def test_conversion_is_reversible():
    clock = ClockOffset()
    for i in range(6):
        clock.add_round(100.0 + i, 90.0 + i, 100.2 + i)
    assert clock.to_phone(clock.to_local(42.0)) == pytest.approx(42.0)


def test_latency_is_reported_once_the_clock_is_locked():
    """M4's acceptance number: under 250 ms and flat over five minutes."""
    streams = Streams()
    for i in range(6):
        streams.clock.add_round(100.0 + i, 100.0 + i, 100.0 + i)
    assert streams.clock.locked
    streams.push_frame(b"x", t_phone=0.0)
    assert streams.latency_ms is not None


def test_health_reports_what_matters():
    streams = Streams()
    streams.push_frame(b"x", t_phone=1.0)
    streams.push_motion({"t": 1.0, "rate_z": 0.0})
    health = streams.health()
    assert set(health) >= {"latency_ms", "frames", "frames_dropped",
                           "drop_rate", "motion_samples"}
