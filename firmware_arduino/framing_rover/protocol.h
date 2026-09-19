/* === COPY - DO NOT EDIT ===============================================
 *
 * Generated from firmware/main/protocol.h by tools/gen_arduino_sketch.py.
 * Edit the original and re-run the generator; an edit here is overwritten
 * and tests/test_arduino_sketch.py will fail in the meantime.
 *
 * The copy exists because Arduino IDE only compiles files beside the .ino.
 * ==================================================================== */

/* protocol.h - wire protocol between the laptop and the rover firmware.
 *
 * SHARED DEFINITION.  This file and app/protocol.py must define the same
 * packets.  Both carry a version byte that is checked on every packet, so a
 * mismatch surfaces as a rejected packet at runtime instead of being misread
 * as a control bug.
 *
 * If you edit one, edit the other, then run:
 *     python tools/gen_test_vectors.py   # regenerates firmware/test/vectors.h
 *     pytest tests/test_protocol.py      # python side
 *     make -C firmware/test              # C side, same vectors
 *
 * W-1  laptop -> firmware, UDP, 14 bytes, little-endian, 20-50 Hz
 * W-2  firmware -> laptop, UDP, 16 bytes, little-endian, 20 Hz
 *
 * Both structs are packed and laid out to match the byte offsets in the PRD
 * exactly, so the on-wire form is a straight memcpy on any little-endian
 * target.  The ESP32-S3 is little-endian; a big-endian host running the unit
 * tests would need the byte-swapping path, which is why the tests compare
 * against byte vectors rather than struct casts.
 */

#ifndef ROVER_PROTOCOL_H
#define ROVER_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Version.  Bump on ANY layout change.  Firmware rejects a mismatch. */
#define PROTOCOL_VERSION 1

#define CMD_SIZE 14
#define TLM_SIZE 16

/* W-1 flags (byte 1 of the command packet) */
#define FLAG_ENABLE       0x01
#define FLAG_TILT_RELEASE 0x02
#define FLAG_ZERO_TILT    0x04

/* W-2 status bits (byte 1 of the telemetry packet) */
#define ST_ENABLED       0x01
#define ST_RANGE_TRIP    0x02
#define ST_VBAT_LOW      0x04
#define ST_WATCHDOG_TRIP 0x08
#define ST_TILT_MOVING   0x10

/* "no range sensor, or no reading this cycle" */
#define RANGE_INVALID 0xFFFFu

#define DUTY_MIN (-1000)
#define DUTY_MAX (1000)

/* ------------------------------------------------------------------ */
/* W-1 laptop -> firmware                                             */
/* ------------------------------------------------------------------ */
typedef struct __attribute__((packed)) {
    uint8_t  version;      /* 0  protocol version, rejected if mismatched   */
    uint8_t  flags;        /* 1  bit0 enable, bit1 tilt_release, bit2 zero  */
    uint16_t seq;          /* 2  increments; firmware ignores non-advancing */
    int16_t  left;         /* 4  -1000..1000 duty                           */
    int16_t  right;        /* 6  -1000..1000 duty                           */
    int16_t  tilt_deg_x10; /* 8  target tilt angle, tenths of a degree      */
    uint16_t reserved;     /* 10                                            */
    uint16_t crc16;        /* 12 over bytes 0..11                           */
} cmd_packet_t;

/* ------------------------------------------------------------------ */
/* W-2 firmware -> laptop                                             */
/* ------------------------------------------------------------------ */
typedef struct __attribute__((packed)) {
    uint8_t  version;    /* 0                                               */
    uint8_t  status;     /* 1  see ST_* bits                                */
    uint16_t seq_echo;   /* 2  last accepted command sequence               */
    int32_t  tilt_steps; /* 4  open-loop step position                      */
    uint16_t range_mm;   /* 8  RANGE_INVALID if no sensor or no reading     */
    uint16_t vbat_mv;    /* 10                                              */
    uint16_t loop_us;    /* 12 worst drive-task period since last report    */
    uint16_t crc16;      /* 14                                              */
} tlm_packet_t;

/* The Arduino build compiles the sketch as C++, where the spelling differs. */
#if defined(__cplusplus)
#define ROVER_STATIC_ASSERT(cond, msg) static_assert(cond, msg)
#else
#define ROVER_STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)
#endif

ROVER_STATIC_ASSERT(sizeof(cmd_packet_t) == CMD_SIZE, "cmd_packet_t must be 14 bytes");
ROVER_STATIC_ASSERT(sizeof(tlm_packet_t) == TLM_SIZE, "tlm_packet_t must be 16 bytes");

/* ------------------------------------------------------------------ */
/* CRC-16/CCITT-FALSE.  poly 0x1021, init 0xFFFF, no reflection,      */
/* no final xor.  Mirrored bit-for-bit by crc16() in protocol.py.     */
/* Check value: crc16("123456789", 9) == 0x29B1.                      */
/* ------------------------------------------------------------------ */
static inline uint16_t proto_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static inline int16_t proto_clamp_duty(int32_t duty)
{
    if (duty < DUTY_MIN) return (int16_t)DUTY_MIN;
    if (duty > DUTY_MAX) return (int16_t)DUTY_MAX;
    return (int16_t)duty;
}

/* W-5.3: monotonic, wraps at 65535.  Half the sequence space counts as
 * "ahead" so a wrap is not mistaken for a stalled sender. */
static inline bool proto_seq_advanced(uint16_t new_seq, uint16_t old_seq)
{
    uint16_t delta = (uint16_t)(new_seq - old_seq);
    return delta != 0u && delta < 0x8000u;
}

/* Validate length, CRC and version, then copy out.  Returns false on any
 * failure; the caller drops the packet and lets the watchdog do its job. */
static inline bool proto_parse_command(const uint8_t *buf, size_t len, cmd_packet_t *out)
{
    if (len != CMD_SIZE || buf == NULL || out == NULL) {
        return false;
    }
    uint16_t want = proto_crc16(buf, CMD_SIZE - 2);
    uint16_t got = (uint16_t)buf[12] | ((uint16_t)buf[13] << 8);
    if (want != got) {
        return false;
    }
    if (buf[0] != PROTOCOL_VERSION) {
        return false;
    }
    memcpy(out, buf, CMD_SIZE);
    return true;
}

/* Fill version and CRC, then serialise.  buf must hold TLM_SIZE bytes. */
static inline void proto_build_telemetry(tlm_packet_t *tlm, uint8_t *buf)
{
    tlm->version = PROTOCOL_VERSION;
    memcpy(buf, tlm, TLM_SIZE);
    uint16_t crc = proto_crc16(buf, TLM_SIZE - 2);
    buf[14] = (uint8_t)(crc & 0xFFu);
    buf[15] = (uint8_t)(crc >> 8);
    tlm->crc16 = crc;
}

static inline void proto_build_command(cmd_packet_t *cmd, uint8_t *buf)
{
    cmd->version = PROTOCOL_VERSION;
    memcpy(buf, cmd, CMD_SIZE);
    uint16_t crc = proto_crc16(buf, CMD_SIZE - 2);
    buf[12] = (uint8_t)(crc & 0xFFu);
    buf[13] = (uint8_t)(crc >> 8);
    cmd->crc16 = crc;
}

#ifdef __cplusplus
}
#endif

#endif /* ROVER_PROTOCOL_H */
