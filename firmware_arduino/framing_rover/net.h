/* === COPY - DO NOT EDIT ===============================================
 *
 * Generated from firmware/main/net.h by tools/gen_arduino_sketch.py.
 * Edit the original and re-run the generator; an edit here is overwritten
 * and tests/test_arduino_sketch.py will fail in the meantime.
 *
 * The copy exists because Arduino IDE only compiles files beside the .ino.
 * ==================================================================== */

/* net.h - Wi-Fi join, UDP rx/tx, sequence handling. */

#ifndef ROVER_NET_H
#define ROVER_NET_H

#include <stdbool.h>
#include <stdint.h>

#include "protocol.h"
#include "safety.h"
#include "tilt.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The last accepted command, published for the control tasks.  Read through
 * net_get_command(), which takes the lock; never touch the fields directly. */
typedef struct {
    int16_t left;
    int16_t right;
    float tilt_deg;
    bool enable;
    bool tilt_release;
    bool zero_tilt;
    uint16_t seq;
    int64_t rx_ms;
} net_command_t;

/* Join Wi-Fi and bind the command socket.  Blocks until the station has an
 * IP, then returns.  The firmware never initiates and never retries into the
 * watchdog's margin (W-5.2, F-5): a reconnect runs in its own task and the
 * watchdog keeps tripping while it does. */
void net_init(void);

/* The two network tasks, pinned to core 0 by main.c. */
void net_rx_task(void *arg);
void net_telemetry_task(void *arg);

/* Snapshot the last accepted command.  Returns false if none has arrived. */
bool net_get_command(net_command_t *out);

/* Counters for the bring-up log. */
typedef struct {
    uint32_t rx_total;
    uint32_t rx_bad_len;
    uint32_t rx_bad_crc;
    uint32_t rx_bad_version;
    uint32_t rx_stale_seq;
    uint32_t rx_accepted;
    uint32_t tx_telemetry;
} net_stats_t;

void net_get_stats(net_stats_t *out);

/* Worst drive-task period since the last telemetry report (W-5.5).  drive
 * task calls the setter; the telemetry task reads and clears. */
void net_report_loop_us(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif /* ROVER_NET_H */
