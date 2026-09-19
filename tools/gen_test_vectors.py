#!/usr/bin/env python3
"""Generate the shared protocol test vectors.

One source of truth for both languages:

    tests/fixtures/protocol_vectors.json   <- written here, read by pytest
    firmware/test/vectors.h                <- written here, read by the C test

The Python implementation in app/protocol.py produces the bytes; the C test
then has to reproduce them.  That asymmetry is deliberate: protocol.py is the
easier file to reason about, so it is the reference, and any drift in
protocol.h shows up as a failing C test rather than as a field mystery.

Run after any change to either protocol file.
"""

from __future__ import annotations

import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from app import protocol as P  # noqa: E402

# Cases are chosen to cover sign, saturation, wrap and every flag/status bit.
COMMAND_CASES = [
    ("zero", dict(seq=0, left=0, right=0, tilt_deg=0.0, enable=False)),
    ("enable_only", dict(seq=1, left=0, right=0, tilt_deg=0.0, enable=True)),
    ("forward", dict(seq=2, left=500, right=500, tilt_deg=0.0, enable=True)),
    ("spin_left", dict(seq=3, left=-400, right=400, tilt_deg=0.0, enable=True)),
    ("negative_tilt", dict(seq=4, left=0, right=0, tilt_deg=-12.3, enable=True)),
    ("positive_tilt", dict(seq=5, left=0, right=0, tilt_deg=31.7, enable=True)),
    ("saturate_high", dict(seq=6, left=1000, right=1000, tilt_deg=90.0, enable=True)),
    ("saturate_low", dict(seq=7, left=-1000, right=-1000, tilt_deg=-90.0, enable=True)),
    ("clamped_over", dict(seq=8, left=32000, right=-32000, tilt_deg=0.0, enable=True)),
    ("tilt_release", dict(seq=9, left=0, right=0, tilt_deg=5.0, enable=True, tilt_release=True)),
    ("zero_tilt", dict(seq=10, left=0, right=0, tilt_deg=0.0, enable=False, zero_tilt=True)),
    ("all_flags", dict(seq=11, left=-1, right=1, tilt_deg=0.1, enable=True,
                       tilt_release=True, zero_tilt=True)),
    ("seq_near_wrap", dict(seq=65534, left=123, right=-123, tilt_deg=-0.1, enable=True)),
    ("seq_wrapped", dict(seq=0, left=321, right=-321, tilt_deg=0.0, enable=True)),
    ("odd_duty", dict(seq=4242, left=317, right=-289, tilt_deg=-7.9, enable=True)),
]

TELEMETRY_CASES = [
    ("idle", dict(status=0, seq_echo=0, tilt_steps=0, range_mm=P.RANGE_INVALID,
                  vbat_mv=0, loop_us=0)),
    ("running", dict(status=P.ST_ENABLED, seq_echo=42, tilt_steps=1024,
                     range_mm=1500, vbat_mv=7400, loop_us=10050)),
    ("range_trip", dict(status=P.ST_ENABLED | P.ST_RANGE_TRIP, seq_echo=43,
                        tilt_steps=-512, range_mm=310, vbat_mv=7380, loop_us=10120)),
    ("vbat_low", dict(status=P.ST_VBAT_LOW, seq_echo=44, tilt_steps=0,
                      range_mm=2000, vbat_mv=6500, loop_us=9980)),
    ("watchdog", dict(status=P.ST_WATCHDOG_TRIP, seq_echo=45, tilt_steps=7,
                      range_mm=P.RANGE_INVALID, vbat_mv=7100, loop_us=25000)),
    ("tilt_moving", dict(status=P.ST_ENABLED | P.ST_TILT_MOVING, seq_echo=46,
                         tilt_steps=-4096, range_mm=900, vbat_mv=7250, loop_us=10011)),
    ("all_bits", dict(status=0x1F, seq_echo=65535, tilt_steps=123456,
                      range_mm=65534, vbat_mv=65535, loop_us=65535)),
    ("negative_steps", dict(status=P.ST_ENABLED, seq_echo=1, tilt_steps=-123456,
                            range_mm=350, vbat_mv=6600, loop_us=1)),
]


def build() -> dict:
    commands = []
    for name, kwargs in COMMAND_CASES:
        cmd = P.Command(**kwargs)
        raw = P.pack_command(cmd)
        parsed = P.unpack_command(raw)
        commands.append({
            "name": name,
            "seq": parsed.seq,
            "left": parsed.left,
            "right": parsed.right,
            "tilt_deg_x10": int(round(parsed.tilt_deg * 10)),
            "enable": parsed.enable,
            "tilt_release": parsed.tilt_release,
            "zero_tilt": parsed.zero_tilt,
            "bytes": raw.hex(),
        })

    telemetry = []
    for name, kwargs in TELEMETRY_CASES:
        tlm = P.Telemetry(**kwargs)
        raw = P.pack_telemetry(tlm)
        parsed = P.unpack_telemetry(raw)
        telemetry.append({
            "name": name,
            "status": parsed.status,
            "seq_echo": parsed.seq_echo,
            "tilt_steps": parsed.tilt_steps,
            "range_mm": parsed.range_mm,
            "vbat_mv": parsed.vbat_mv,
            "loop_us": parsed.loop_us,
            "bytes": raw.hex(),
        })

    crc_cases = [
        {"input": "123456789", "crc": P.crc16(b"123456789")},
        {"input": "", "crc": P.crc16(b"")},
        {"input": "A", "crc": P.crc16(b"A")},
        {"input": "rover", "crc": P.crc16(b"rover")},
        {"input": "\x00\x00\x00\x00", "crc": P.crc16(b"\x00\x00\x00\x00")},
        {"input": "\xff\xff\xff\xff", "crc": P.crc16(b"\xff\xff\xff\xff")},
    ]

    seq_cases = [
        {"new": 1, "old": 0, "advanced": P.seq_advanced(1, 0)},
        {"new": 0, "old": 0, "advanced": P.seq_advanced(0, 0)},
        {"new": 0, "old": 65535, "advanced": P.seq_advanced(0, 65535)},
        {"new": 65535, "old": 0, "advanced": P.seq_advanced(65535, 0)},
        {"new": 100, "old": 99, "advanced": P.seq_advanced(100, 99)},
        {"new": 99, "old": 100, "advanced": P.seq_advanced(99, 100)},
        {"new": 5, "old": 65530, "advanced": P.seq_advanced(5, 65530)},
    ]

    return {
        "protocol_version": P.PROTOCOL_VERSION,
        "cmd_size": P.CMD_SIZE,
        "tlm_size": P.TLM_SIZE,
        "crc": crc_cases,
        "seq": seq_cases,
        "commands": commands,
        "telemetry": telemetry,
    }


def c_bytes(hexstr: str) -> str:
    raw = bytes.fromhex(hexstr)
    return "{" + ", ".join(f"0x{b:02X}" for b in raw) + "}"


def c_escape(s: str) -> str:
    return "".join(f"\\x{ord(ch):02x}" for ch in s)


def write_header(data: dict, path: pathlib.Path) -> None:
    lines = [
        "/* vectors.h - GENERATED by tools/gen_test_vectors.py.  Do not edit. */",
        "/* Shared with tests/fixtures/protocol_vectors.json; the Python",
        "   implementation is the reference and the C side must reproduce it. */",
        "#ifndef ROVER_TEST_VECTORS_H",
        "#define ROVER_TEST_VECTORS_H",
        "",
        "#include <stdbool.h>",
        "#include <stdint.h>",
        "",
        f"#define VEC_PROTOCOL_VERSION {data['protocol_version']}",
        f"#define VEC_CMD_SIZE {data['cmd_size']}",
        f"#define VEC_TLM_SIZE {data['tlm_size']}",
        "",
        "typedef struct { const char *input; int len; uint16_t crc; } crc_vector_t;",
        "typedef struct { uint16_t new_seq, old_seq; bool advanced; } seq_vector_t;",
        "typedef struct {",
        "    const char *name; uint16_t seq; int16_t left, right, tilt_deg_x10;",
        "    bool enable, tilt_release, zero_tilt; uint8_t bytes[14];",
        "} cmd_vector_t;",
        "typedef struct {",
        "    const char *name; uint8_t status; uint16_t seq_echo; int32_t tilt_steps;",
        "    uint16_t range_mm, vbat_mv, loop_us; uint8_t bytes[16];",
        "} tlm_vector_t;",
        "",
        "static const crc_vector_t CRC_VECTORS[] = {",
    ]
    for c in data["crc"]:
        lines.append(f'    {{"{c_escape(c["input"])}", {len(c["input"])}, 0x{c["crc"]:04X}}},')
    lines += [
        "};",
        "#define N_CRC_VECTORS (sizeof(CRC_VECTORS)/sizeof(CRC_VECTORS[0]))",
        "",
        "static const seq_vector_t SEQ_VECTORS[] = {",
    ]
    for s in data["seq"]:
        lines.append(f'    {{{s["new"]}, {s["old"]}, {"true" if s["advanced"] else "false"}}},')
    lines += [
        "};",
        "#define N_SEQ_VECTORS (sizeof(SEQ_VECTORS)/sizeof(SEQ_VECTORS[0]))",
        "",
        "static const cmd_vector_t CMD_VECTORS[] = {",
    ]
    for c in data["commands"]:
        lines.append(
            f'    {{"{c["name"]}", {c["seq"]}, {c["left"]}, {c["right"]}, '
            f'{c["tilt_deg_x10"]}, {"true" if c["enable"] else "false"}, '
            f'{"true" if c["tilt_release"] else "false"}, '
            f'{"true" if c["zero_tilt"] else "false"}, {c_bytes(c["bytes"])}}},'
        )
    lines += [
        "};",
        "#define N_CMD_VECTORS (sizeof(CMD_VECTORS)/sizeof(CMD_VECTORS[0]))",
        "",
        "static const tlm_vector_t TLM_VECTORS[] = {",
    ]
    for t in data["telemetry"]:
        lines.append(
            f'    {{"{t["name"]}", {t["status"]}, {t["seq_echo"]}, {t["tilt_steps"]}, '
            f'{t["range_mm"]}, {t["vbat_mv"]}, {t["loop_us"]}, {c_bytes(t["bytes"])}}},'
        )
    lines += [
        "};",
        "#define N_TLM_VECTORS (sizeof(TLM_VECTORS)/sizeof(TLM_VECTORS[0]))",
        "",
        "#endif /* ROVER_TEST_VECTORS_H */",
        "",
    ]
    path.write_text("\n".join(lines))


def main() -> None:
    data = build()
    json_path = ROOT / "tests" / "fixtures" / "protocol_vectors.json"
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(json.dumps(data, indent=2) + "\n")

    header_path = ROOT / "firmware" / "test" / "vectors.h"
    header_path.parent.mkdir(parents=True, exist_ok=True)
    write_header(data, header_path)

    print(f"wrote {json_path.relative_to(ROOT)}  "
          f"({len(data['commands'])} commands, {len(data['telemetry'])} telemetry)")
    print(f"wrote {header_path.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
