/* test_protocol.c - the C half of the shared protocol test.
 *
 * Both languages are checked against the SAME vectors (firmware/test/vectors.h
 * and tests/fixtures/protocol_vectors.json, generated together by
 * tools/gen_test_vectors.py).  A change to protocol.h that protocol.py did not
 * get shows up here, at a desk, instead of as a silent control bug in a field.
 */

#include <stdio.h>
#include <string.h>

#include "protocol.h"
#include "test_util.h"
#include "vectors.h"

int g_tests_run = 0;
int g_tests_failed = 0;
const char *g_current_test = NULL;

static void test_sizes_match_prd(void)
{
    CHECK_EQ_INT(sizeof(cmd_packet_t), 14);
    CHECK_EQ_INT(sizeof(tlm_packet_t), 16);
    CHECK_EQ_INT(CMD_SIZE, VEC_CMD_SIZE);
    CHECK_EQ_INT(TLM_SIZE, VEC_TLM_SIZE);
    CHECK_EQ_INT(PROTOCOL_VERSION, VEC_PROTOCOL_VERSION);
}

static void test_field_offsets(void)
{
    /* W-1 offsets, read straight off the PRD table. */
    cmd_packet_t c;
    const char *base = (const char *)&c;
    CHECK_EQ_INT((const char *)&c.version - base, 0);
    CHECK_EQ_INT((const char *)&c.flags - base, 1);
    CHECK_EQ_INT((const char *)&c.seq - base, 2);
    CHECK_EQ_INT((const char *)&c.left - base, 4);
    CHECK_EQ_INT((const char *)&c.right - base, 6);
    CHECK_EQ_INT((const char *)&c.tilt_deg_x10 - base, 8);
    CHECK_EQ_INT((const char *)&c.reserved - base, 10);
    CHECK_EQ_INT((const char *)&c.crc16 - base, 12);

    /* W-2 offsets. */
    tlm_packet_t t;
    const char *tb = (const char *)&t;
    CHECK_EQ_INT((const char *)&t.version - tb, 0);
    CHECK_EQ_INT((const char *)&t.status - tb, 1);
    CHECK_EQ_INT((const char *)&t.seq_echo - tb, 2);
    CHECK_EQ_INT((const char *)&t.tilt_steps - tb, 4);
    CHECK_EQ_INT((const char *)&t.range_mm - tb, 8);
    CHECK_EQ_INT((const char *)&t.vbat_mv - tb, 10);
    CHECK_EQ_INT((const char *)&t.loop_us - tb, 12);
    CHECK_EQ_INT((const char *)&t.crc16 - tb, 14);
}

static void test_crc_vectors(void)
{
    for (size_t i = 0; i < N_CRC_VECTORS; i++) {
        const crc_vector_t *v = &CRC_VECTORS[i];
        uint16_t got = proto_crc16((const uint8_t *)v->input, (size_t)v->len);
        if (got != v->crc) {
            printf("  crc vector %zu: got 0x%04X, expected 0x%04X\n", i, got, v->crc);
        }
        CHECK_EQ_INT(got, v->crc);
    }
    /* The published CRC-16/CCITT-FALSE check value. */
    CHECK_EQ_INT(proto_crc16((const uint8_t *)"123456789", 9), 0x29B1);
}

static void test_seq_vectors(void)
{
    for (size_t i = 0; i < N_SEQ_VECTORS; i++) {
        const seq_vector_t *v = &SEQ_VECTORS[i];
        bool got = proto_seq_advanced(v->new_seq, v->old_seq);
        if (got != v->advanced) {
            printf("  seq %u after %u: got %d, expected %d\n",
                   v->new_seq, v->old_seq, (int)got, (int)v->advanced);
        }
        CHECK_EQ_INT((int)got, (int)v->advanced);
    }
}

static void test_command_parse_matches_python(void)
{
    for (size_t i = 0; i < N_CMD_VECTORS; i++) {
        const cmd_vector_t *v = &CMD_VECTORS[i];
        cmd_packet_t pkt;
        bool ok = proto_parse_command(v->bytes, CMD_SIZE, &pkt);
        if (!ok) {
            printf("  command vector '%s' failed to parse\n", v->name);
        }
        CHECK(ok);
        CHECK_EQ_INT(pkt.seq, v->seq);
        CHECK_EQ_INT(pkt.left, v->left);
        CHECK_EQ_INT(pkt.right, v->right);
        CHECK_EQ_INT(pkt.tilt_deg_x10, v->tilt_deg_x10);
        CHECK_EQ_INT((pkt.flags & FLAG_ENABLE) != 0, v->enable);
        CHECK_EQ_INT((pkt.flags & FLAG_TILT_RELEASE) != 0, v->tilt_release);
        CHECK_EQ_INT((pkt.flags & FLAG_ZERO_TILT) != 0, v->zero_tilt);
    }
}

static void test_command_build_matches_python(void)
{
    /* The other direction: build the packet in C and compare bytes with the
     * Python output.  This is what catches a struct padding surprise. */
    for (size_t i = 0; i < N_CMD_VECTORS; i++) {
        const cmd_vector_t *v = &CMD_VECTORS[i];
        cmd_packet_t pkt = {
            .version = PROTOCOL_VERSION,
            .flags = (uint8_t)((v->enable ? FLAG_ENABLE : 0) |
                               (v->tilt_release ? FLAG_TILT_RELEASE : 0) |
                               (v->zero_tilt ? FLAG_ZERO_TILT : 0)),
            .seq = v->seq,
            .left = v->left,
            .right = v->right,
            .tilt_deg_x10 = v->tilt_deg_x10,
            .reserved = 0,
            .crc16 = 0,
        };
        uint8_t buf[CMD_SIZE];
        proto_build_command(&pkt, buf);
        if (memcmp(buf, v->bytes, CMD_SIZE) != 0) {
            printf("  command vector '%s' bytes differ\n", v->name);
        }
        CHECK_EQ_MEM(buf, v->bytes, CMD_SIZE);
    }
}

static void test_telemetry_build_matches_python(void)
{
    for (size_t i = 0; i < N_TLM_VECTORS; i++) {
        const tlm_vector_t *v = &TLM_VECTORS[i];
        tlm_packet_t pkt = {
            .version = PROTOCOL_VERSION,
            .status = v->status,
            .seq_echo = v->seq_echo,
            .tilt_steps = v->tilt_steps,
            .range_mm = v->range_mm,
            .vbat_mv = v->vbat_mv,
            .loop_us = v->loop_us,
            .crc16 = 0,
        };
        uint8_t buf[TLM_SIZE];
        proto_build_telemetry(&pkt, buf);
        if (memcmp(buf, v->bytes, TLM_SIZE) != 0) {
            printf("  telemetry vector '%s' bytes differ\n", v->name);
        }
        CHECK_EQ_MEM(buf, v->bytes, TLM_SIZE);
    }
}

static void test_rejects_bad_crc(void)
{
    uint8_t buf[CMD_SIZE];
    memcpy(buf, CMD_VECTORS[2].bytes, CMD_SIZE);
    buf[4] ^= 0x01;   /* flip a bit in `left` without fixing the CRC */
    cmd_packet_t pkt;
    CHECK(!proto_parse_command(buf, CMD_SIZE, &pkt));
}

static void test_rejects_bad_version(void)
{
    cmd_packet_t pkt = {.version = PROTOCOL_VERSION + 1, .seq = 1};
    uint8_t buf[CMD_SIZE];
    /* Build with a good CRC but the wrong version - exactly what a half-updated
     * protocol.py would send. */
    memcpy(buf, &pkt, CMD_SIZE);
    buf[0] = PROTOCOL_VERSION + 1;
    uint16_t crc = proto_crc16(buf, CMD_SIZE - 2);
    buf[12] = (uint8_t)(crc & 0xFF);
    buf[13] = (uint8_t)(crc >> 8);

    cmd_packet_t out;
    CHECK(!proto_parse_command(buf, CMD_SIZE, &out));
}

static void test_rejects_bad_length(void)
{
    cmd_packet_t pkt;
    CHECK(!proto_parse_command(CMD_VECTORS[0].bytes, CMD_SIZE - 1, &pkt));
    CHECK(!proto_parse_command(CMD_VECTORS[0].bytes, CMD_SIZE + 1, &pkt));
}

static void test_clamp_duty(void)
{
    CHECK_EQ_INT(proto_clamp_duty(0), 0);
    CHECK_EQ_INT(proto_clamp_duty(1000), 1000);
    CHECK_EQ_INT(proto_clamp_duty(-1000), -1000);
    CHECK_EQ_INT(proto_clamp_duty(32000), 1000);
    CHECK_EQ_INT(proto_clamp_duty(-32000), -1000);
}

int main(void)
{
    printf("protocol (C side, shared vectors)\n");
    RUN(test_sizes_match_prd);
    RUN(test_field_offsets);
    RUN(test_crc_vectors);
    RUN(test_seq_vectors);
    RUN(test_command_parse_matches_python);
    RUN(test_command_build_matches_python);
    RUN(test_telemetry_build_matches_python);
    RUN(test_rejects_bad_crc);
    RUN(test_rejects_bad_version);
    RUN(test_rejects_bad_length);
    RUN(test_clamp_duty);
    TEST_MAIN_EPILOGUE();
}
