"""Wire protocol between the laptop and the rover firmware.

SHARED DEFINITION.  This file and ``firmware/main/protocol.h`` must define the
same packets.  Both carry a version byte that is checked on every packet, so a
mismatch surfaces as a rejected packet at runtime instead of being misread as a
control bug.

If you edit one, edit the other, then run::

    python tools/gen_test_vectors.py     # regenerates firmware/test/vectors.h
    pytest tests/test_protocol.py        # python side
    make -C firmware/test                # C side, same vectors

W-1  laptop -> firmware, UDP, 14 bytes, little-endian, 20-50 Hz
W-2  firmware -> laptop, UDP, 16 bytes, little-endian, 20 Hz
"""

from __future__ import annotations

import struct
from typing import NamedTuple

# --------------------------------------------------------------------------
# Version.  Bump on ANY layout change.  Firmware rejects a mismatched version.
# --------------------------------------------------------------------------
PROTOCOL_VERSION = 1

CMD_SIZE = 14
TLM_SIZE = 16

# W-1 flags (byte 1 of the command packet)
FLAG_ENABLE = 0x01
FLAG_TILT_RELEASE = 0x02
FLAG_ZERO_TILT = 0x04

# W-2 status bits (byte 1 of the telemetry packet)
ST_ENABLED = 0x01
ST_RANGE_TRIP = 0x02
ST_VBAT_LOW = 0x04
ST_WATCHDOG_TRIP = 0x08
ST_TILT_MOVING = 0x10

# Sentinel for "no range sensor, or no reading this cycle"
RANGE_INVALID = 0xFFFF

DUTY_MIN = -1000
DUTY_MAX = 1000

_CMD_BODY = struct.Struct("<BBHhhhH")   # 12 bytes, bytes 0..11
_CMD_CRC = struct.Struct("<H")          # bytes 12..13
_TLM_BODY = struct.Struct("<BBHiHHH")   # 14 bytes, bytes 0..13
_TLM_CRC = struct.Struct("<H")          # bytes 14..15

assert _CMD_BODY.size + _CMD_CRC.size == CMD_SIZE
assert _TLM_BODY.size + _TLM_CRC.size == TLM_SIZE


class ProtocolError(ValueError):
    """Raised on a bad length, CRC or version.  Callers drop the packet."""


# --------------------------------------------------------------------------
# CRC-16/CCITT-FALSE.  poly 0x1021, init 0xFFFF, no reflection, no final xor.
# Mirrored bit-for-bit by crc16() in protocol.h.
# --------------------------------------------------------------------------
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def clamp_duty(duty: int) -> int:
    """Clamp to the -1000..1000 the wire format and firmware both assume."""
    return max(DUTY_MIN, min(DUTY_MAX, int(duty)))


def seq_advanced(new: int, old: int) -> bool:
    """True if ``new`` is ahead of ``old`` in a 16-bit wrapping sequence.

    W-5.3: sequence numbers are monotonic and wrap at 65535.  The firmware
    rejects a non-advancing sequence, which makes a stuck sender
    indistinguishable from silence.  Half the sequence space is treated as
    "ahead" so a wrap is not mistaken for a stall.
    """
    return ((new - old) & 0xFFFF) != 0 and ((new - old) & 0xFFFF) < 0x8000


class Command(NamedTuple):
    """W-1 laptop -> firmware."""

    seq: int
    left: int              # -1000..1000 duty
    right: int             # -1000..1000 duty
    tilt_deg: float        # target tilt angle, degrees
    enable: bool
    tilt_release: bool = False
    zero_tilt: bool = False


class Telemetry(NamedTuple):
    """W-2 firmware -> laptop."""

    status: int
    seq_echo: int
    tilt_steps: int
    range_mm: int
    vbat_mv: int
    loop_us: int

    @property
    def enabled(self) -> bool:
        return bool(self.status & ST_ENABLED)

    @property
    def range_trip(self) -> bool:
        return bool(self.status & ST_RANGE_TRIP)

    @property
    def vbat_low(self) -> bool:
        return bool(self.status & ST_VBAT_LOW)

    @property
    def watchdog_trip(self) -> bool:
        return bool(self.status & ST_WATCHDOG_TRIP)

    @property
    def tilt_moving(self) -> bool:
        return bool(self.status & ST_TILT_MOVING)

    @property
    def range_valid(self) -> bool:
        return self.range_mm != RANGE_INVALID


def pack_command(cmd: Command) -> bytes:
    """Serialise W-1.  Duty is clamped; tilt is quantised to tenths of a degree."""
    flags = 0
    if cmd.enable:
        flags |= FLAG_ENABLE
    if cmd.tilt_release:
        flags |= FLAG_TILT_RELEASE
    if cmd.zero_tilt:
        flags |= FLAG_ZERO_TILT

    tilt_x10 = int(round(cmd.tilt_deg * 10.0))
    tilt_x10 = max(-32768, min(32767, tilt_x10))

    body = _CMD_BODY.pack(
        PROTOCOL_VERSION,
        flags,
        cmd.seq & 0xFFFF,
        clamp_duty(cmd.left),
        clamp_duty(cmd.right),
        tilt_x10,
        0,  # reserved
    )
    return body + _CMD_CRC.pack(crc16(body))


def unpack_command(buf: bytes) -> Command:
    """Parse W-1.  Raises ProtocolError on length, version or CRC failure."""
    if len(buf) != CMD_SIZE:
        raise ProtocolError(f"command packet is {len(buf)} bytes, expected {CMD_SIZE}")
    body, tail = buf[:_CMD_BODY.size], buf[_CMD_BODY.size:]
    (got_crc,) = _CMD_CRC.unpack(tail)
    want_crc = crc16(body)
    if got_crc != want_crc:
        raise ProtocolError(f"command crc 0x{got_crc:04X}, computed 0x{want_crc:04X}")
    version, flags, seq, left, right, tilt_x10, _reserved = _CMD_BODY.unpack(body)
    if version != PROTOCOL_VERSION:
        raise ProtocolError(f"command version {version}, expected {PROTOCOL_VERSION}")
    return Command(
        seq=seq,
        left=left,
        right=right,
        tilt_deg=tilt_x10 / 10.0,
        enable=bool(flags & FLAG_ENABLE),
        tilt_release=bool(flags & FLAG_TILT_RELEASE),
        zero_tilt=bool(flags & FLAG_ZERO_TILT),
    )


def pack_telemetry(tlm: Telemetry) -> bytes:
    """Serialise W-2."""
    body = _TLM_BODY.pack(
        PROTOCOL_VERSION,
        tlm.status & 0xFF,
        tlm.seq_echo & 0xFFFF,
        int(tlm.tilt_steps),
        tlm.range_mm & 0xFFFF,
        tlm.vbat_mv & 0xFFFF,
        min(0xFFFF, max(0, int(tlm.loop_us))),
    )
    return body + _TLM_CRC.pack(crc16(body))


def unpack_telemetry(buf: bytes) -> Telemetry:
    """Parse W-2.  Raises ProtocolError on length, version or CRC failure."""
    if len(buf) != TLM_SIZE:
        raise ProtocolError(f"telemetry packet is {len(buf)} bytes, expected {TLM_SIZE}")
    body, tail = buf[:_TLM_BODY.size], buf[_TLM_BODY.size:]
    (got_crc,) = _TLM_CRC.unpack(tail)
    want_crc = crc16(body)
    if got_crc != want_crc:
        raise ProtocolError(f"telemetry crc 0x{got_crc:04X}, computed 0x{want_crc:04X}")
    version, status, seq_echo, tilt_steps, range_mm, vbat_mv, loop_us = _TLM_BODY.unpack(body)
    if version != PROTOCOL_VERSION:
        raise ProtocolError(f"telemetry version {version}, expected {PROTOCOL_VERSION}")
    return Telemetry(
        status=status,
        seq_echo=seq_echo,
        tilt_steps=tilt_steps,
        range_mm=range_mm,
        vbat_mv=vbat_mv,
        loop_us=loop_us,
    )


def status_words(status: int) -> list[str]:
    """Human-readable status bits, for logs and the teleop display."""
    names = [
        (ST_ENABLED, "enabled"),
        (ST_RANGE_TRIP, "range_trip"),
        (ST_VBAT_LOW, "vbat_low"),
        (ST_WATCHDOG_TRIP, "watchdog_trip"),
        (ST_TILT_MOVING, "tilt_moving"),
    ]
    return [name for bit, name in names if status & bit]
