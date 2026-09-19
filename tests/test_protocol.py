"""protocol.py: CRC and packet round-trip against the SAME fixtures the C side
uses (named in the PRD's unit-test list).

``tests/fixtures/protocol_vectors.json`` and ``firmware/test/vectors.h`` are
generated together by ``tools/gen_test_vectors.py``.  If this file passes and
``make -C firmware/test`` fails, protocol.h has drifted from protocol.py - the
PRD's "most likely source of silent misbehaviour", caught at a desk.
"""

import json
import pathlib

import pytest

from app import protocol as P

ROOT = pathlib.Path(__file__).resolve().parents[1]
VECTORS = json.loads((ROOT / "tests" / "fixtures" / "protocol_vectors.json").read_text())


# ---------------------------------------------------------------------------
# the shared vectors
# ---------------------------------------------------------------------------

def test_vectors_match_the_current_protocol_version():
    assert VECTORS["protocol_version"] == P.PROTOCOL_VERSION, \
        "the fixtures are stale - run python tools/gen_test_vectors.py"
    assert VECTORS["cmd_size"] == P.CMD_SIZE
    assert VECTORS["tlm_size"] == P.TLM_SIZE


@pytest.mark.parametrize("case", VECTORS["crc"], ids=lambda c: repr(c["input"])[:16])
def test_crc_vectors(case):
    assert P.crc16(case["input"].encode("latin-1")) == case["crc"]


def test_crc_check_value():
    """The published CRC-16/CCITT-FALSE check value.  If this is right, the
    polynomial, the init value and the reflection settings are all right."""
    assert P.crc16(b"123456789") == 0x29B1


@pytest.mark.parametrize("case", VECTORS["seq"],
                         ids=lambda c: f"{c['new']}after{c['old']}")
def test_sequence_vectors(case):
    assert P.seq_advanced(case["new"], case["old"]) == case["advanced"]


@pytest.mark.parametrize("case", VECTORS["commands"], ids=lambda c: c["name"])
def test_command_vectors_round_trip(case):
    raw = bytes.fromhex(case["bytes"])
    assert len(raw) == P.CMD_SIZE

    parsed = P.unpack_command(raw)
    assert parsed.seq == case["seq"]
    assert parsed.left == case["left"]
    assert parsed.right == case["right"]
    assert round(parsed.tilt_deg * 10) == case["tilt_deg_x10"]
    assert parsed.enable == case["enable"]
    assert parsed.tilt_release == case["tilt_release"]
    assert parsed.zero_tilt == case["zero_tilt"]

    assert P.pack_command(parsed) == raw, "re-packing produced different bytes"


@pytest.mark.parametrize("case", VECTORS["telemetry"], ids=lambda c: c["name"])
def test_telemetry_vectors_round_trip(case):
    raw = bytes.fromhex(case["bytes"])
    assert len(raw) == P.TLM_SIZE

    parsed = P.unpack_telemetry(raw)
    assert parsed.status == case["status"]
    assert parsed.seq_echo == case["seq_echo"]
    assert parsed.tilt_steps == case["tilt_steps"]
    assert parsed.range_mm == case["range_mm"]
    assert parsed.vbat_mv == case["vbat_mv"]
    assert parsed.loop_us == case["loop_us"]

    assert P.pack_telemetry(parsed) == raw


# ---------------------------------------------------------------------------
# W-5.3  sequence handling
# ---------------------------------------------------------------------------

def test_a_non_advancing_sequence_is_rejected():
    """W-5.3: the firmware rejects a non-advancing sequence, which makes a
    stuck sender indistinguishable from silence."""
    assert not P.seq_advanced(5, 5)
    assert not P.seq_advanced(4, 5)


def test_the_sequence_wraps_without_looking_like_a_stall():
    """Monotonic, wrapping at 65535.  The wrap must read as 'advanced', or the
    watchdog would fire once every 65535 packets."""
    assert P.seq_advanced(0, 65535)
    assert P.seq_advanced(5, 65530)
    assert not P.seq_advanced(65530, 5)


def test_half_the_sequence_space_is_the_dividing_line():
    assert P.seq_advanced(0x7FFF, 0)
    assert not P.seq_advanced(0x8000, 0)


# ---------------------------------------------------------------------------
# rejection paths
# ---------------------------------------------------------------------------

def test_a_corrupt_packet_is_rejected():
    raw = bytearray(P.pack_command(P.Command(seq=1, left=100, right=100,
                                             tilt_deg=0.0, enable=True)))
    raw[4] ^= 0x01
    with pytest.raises(P.ProtocolError, match="crc"):
        P.unpack_command(bytes(raw))


def test_a_version_mismatch_is_rejected_not_misread():
    """Both files carry a version byte so a mismatch is caught at runtime
    rather than misread as a control bug."""
    raw = bytearray(P.pack_command(P.Command(seq=1, left=0, right=0,
                                             tilt_deg=0.0, enable=False)))
    raw[0] = P.PROTOCOL_VERSION + 1
    crc = P.crc16(bytes(raw[:12]))
    raw[12], raw[13] = crc & 0xFF, crc >> 8
    with pytest.raises(P.ProtocolError, match="version"):
        P.unpack_command(bytes(raw))


@pytest.mark.parametrize("length", [0, 1, 13, 15, 64])
def test_a_wrong_length_packet_is_rejected(length):
    with pytest.raises(P.ProtocolError):
        P.unpack_command(b"\x00" * length)


def test_telemetry_rejections_mirror_command_rejections():
    raw = bytearray(P.pack_telemetry(P.Telemetry(
        status=1, seq_echo=1, tilt_steps=0, range_mm=1000, vbat_mv=7400, loop_us=10)))
    raw[8] ^= 0xFF
    with pytest.raises(P.ProtocolError):
        P.unpack_telemetry(bytes(raw))


# ---------------------------------------------------------------------------
# clamping
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("duty,expected", [
    (0, 0), (1000, 1000), (-1000, -1000),
    (1001, 1000), (-1001, -1000), (99999, 1000), (-99999, -1000),
])
def test_duty_is_clamped_to_the_wire_range(duty, expected):
    assert P.clamp_duty(duty) == expected


def test_packing_clamps_rather_than_wrapping():
    """A control bug that produced duty 40000 must come out as full scale, not
    as a small negative number after a 16-bit wrap."""
    raw = P.pack_command(P.Command(seq=1, left=40000, right=-40000,
                                   tilt_deg=0.0, enable=True))
    parsed = P.unpack_command(raw)
    assert parsed.left == 1000
    assert parsed.right == -1000


def test_extreme_tilt_angles_saturate_rather_than_wrap():
    raw = P.pack_command(P.Command(seq=1, left=0, right=0,
                                   tilt_deg=9000.0, enable=False))
    assert P.unpack_command(raw).tilt_deg == pytest.approx(3276.7)


def test_tilt_is_quantised_to_tenths_of_a_degree():
    raw = P.pack_command(P.Command(seq=1, left=0, right=0,
                                   tilt_deg=-12.34, enable=False))
    assert P.unpack_command(raw).tilt_deg == pytest.approx(-12.3)


# ---------------------------------------------------------------------------
# status bits
# ---------------------------------------------------------------------------

def test_status_bits_decode():
    tlm = P.Telemetry(status=P.ST_ENABLED | P.ST_RANGE_TRIP, seq_echo=0,
                      tilt_steps=0, range_mm=300, vbat_mv=7400, loop_us=10)
    assert tlm.enabled and tlm.range_trip
    assert not tlm.vbat_low and not tlm.watchdog_trip and not tlm.tilt_moving
    assert set(P.status_words(tlm.status)) == {"enabled", "range_trip"}


def test_the_no_reading_sentinel_is_not_a_distance():
    """0xFFFF means "no sensor or no reading", and must never be treated as
    65 metres of clear ground."""
    tlm = P.Telemetry(status=0, seq_echo=0, tilt_steps=0,
                      range_mm=P.RANGE_INVALID, vbat_mv=7400, loop_us=10)
    assert not tlm.range_valid


def test_every_flag_survives_the_round_trip():
    cmd = P.Command(seq=7, left=1, right=-1, tilt_deg=0.1, enable=True,
                    tilt_release=True, zero_tilt=True)
    assert P.unpack_command(P.pack_command(cmd)) == cmd
