/* === COPY - DO NOT EDIT ===============================================
 *
 * Generated from firmware/main/safety.h by tools/gen_arduino_sketch.py.
 * Edit the original and re-run the generator; an edit here is overwritten
 * and tests/test_arduino_sketch.py will fail in the meantime.
 *
 * The copy exists because Arduino IDE only compiles files beside the .ino.
 * ==================================================================== */

/* safety.h - watchdog, range stop, enable gate.
 *
 * Design rule 4: safety lives in firmware.  Every stop path in this file must
 * work with the laptop switched off.
 */

#ifndef ROVER_SAFETY_H
#define ROVER_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int watchdog_ms;
    int range_stop_mm;
    int vbat_min_mv;
} safety_cfg_t;

typedef struct {
    bool watchdog_trip;
    bool range_trip;
    bool vbat_low;
    uint16_t range_mm;
    uint16_t vbat_mv;
} safety_status_t;

void safety_init(const safety_cfg_t *cfg);

/* Called by net.c on every ACCEPTED command packet (CRC ok, version ok,
 * sequence advanced).  A packet that fails any of those does not feed the
 * watchdog - a stuck sender is indistinguishable from silence (W-5.3). */
void safety_feed(int64_t now_ms);

/* One 100 Hz step.  Reads the range sensor and battery and updates the trips. */
void safety_tick(void);

void safety_get(safety_status_t *out);

/* Returns the status bits for telemetry (ST_* from protocol.h). */
uint8_t safety_status_bits(bool enabled, bool tilt_moving);

/* ---- pure logic, exposed for unit tests ---- */

/* F-4: a range trip refuses positive (forward) duty while still allowing
 * reverse and turns.  Clamping each side independently would turn a spin into
 * a one-sided arc, so the split is done on the common/differential pair: the
 * forward component is removed and the turning component is left untouched. */
void safety_limit_forward(int left_in, int right_in, int *left_out, int *right_out);

/* True when the command may be applied at all. */
bool safety_motion_allowed(bool enabled, const safety_status_t *st);

#ifdef __cplusplus
}
#endif

#endif /* ROVER_SAFETY_H */
