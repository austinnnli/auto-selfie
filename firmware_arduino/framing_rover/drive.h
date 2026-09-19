/* === COPY - DO NOT EDIT ===============================================
 *
 * Generated from firmware/main/drive.h by tools/gen_arduino_sketch.py.
 * Edit the original and re-run the generator; an edit here is overwritten
 * and tests/test_arduino_sketch.py will fail in the meantime.
 *
 * The copy exists because Arduino IDE only compiles files beside the .ino.
 * ==================================================================== */

/* drive.h - L298N: signed duty per side -> PWM + direction pins. */

#ifndef ROVER_DRIVE_H
#define ROVER_DRIVE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int min_duty_left;
    int min_duty_right;
    int max_duty;
    int max_duty_per_tick;
    float trim;          /* -1..1, positive steers right on a straight run */
    int pwm_freq;
} drive_cfg_t;

/* Applied state, exposed for telemetry and tests. */
typedef struct {
    int applied_left;    /* what is actually on the pins right now */
    int applied_right;
    int target_left;     /* what the last accepted command asked for */
    int target_right;
    bool coasting;
} drive_state_t;

void drive_init(const drive_cfg_t *cfg);

/* Set the requested duty per side, -1000..1000.  Nothing reaches the pins
 * until the next drive_tick(). */
void drive_set(int left, int right);

/* One 100 Hz step of ramping and application (F-2). */
void drive_tick(void);

/* F-2.5: duty 0 and both direction pins low.  Used on enable == false and on
 * any safety trip.  Takes effect immediately, not on the next tick. */
void drive_coast(void);

void drive_get_state(drive_state_t *out);

/* ---- pure logic, exposed for unit tests (test/test_drive_tilt.c) ---- */

/* F-2.1 / D-2: an L298N bridge turns nothing below roughly 25-40% duty, so a
 * small proportional output would be silently discarded and the loop would
 * appear dead near its target.  Any non-zero request is lifted to at least
 * min_duty.  Idempotent: a value already in the live band is unchanged, so it
 * is safe for the laptop to have done the same mapping already. */
int drive_apply_min_duty(int duty, int min_duty, int max_duty);

/* F-2.2: limit current surge by ramping toward the request. */
int drive_ramp(int current, int target, int max_delta);

/* F-2.1: trim is a mechanical property of the chassis, corrected here rather
 * than in the control law.  Positive trim slows the left side. */
void drive_apply_trim(int left_in, int right_in, float trim, int *left_out, int *right_out);

#ifdef __cplusplus
}
#endif

#endif /* ROVER_DRIVE_H */
