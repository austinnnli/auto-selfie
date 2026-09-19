/* === COPY - DO NOT EDIT ===============================================
 *
 * Generated from firmware/main/tilt.h by tools/gen_arduino_sketch.py.
 * Edit the original and re-run the generator; an edit here is overwritten
 * and tests/test_arduino_sketch.py will fail in the meantime.
 *
 * The copy exists because Arduino IDE only compiles files beside the .ino.
 * ==================================================================== */

/* tilt.h - 28BYJ-48 half-step sequencing with backlash compensation. */

#ifndef ROVER_TILT_H
#define ROVER_TILT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float steps_per_degree;
    int backlash_steps;
    int max_step_rate;    /* half-steps per second */
    int release_ms;       /* de-energise this long after the last step */
    float min_deg;
    float max_deg;
} tilt_cfg_t;

typedef struct {
    int32_t position;     /* open-loop half-step count, telemetry only */
    int32_t target;       /* final half-step target */
    int32_t waypoint;     /* where we are heading right now (overshoot or final) */
    bool moving;
    bool energised;
} tilt_state_t;

void tilt_init(const tilt_cfg_t *cfg);

/* F-3.2: the target arrives as an ANGLE, not a step count.  The firmware's
 * step position is an open-loop guess; the laptop closes the loop against the
 * phone's gravity vector and sends a corrected angle (D-9). */
void tilt_set_target_deg(float deg);

/* F-3: one 1 kHz tick.  Advances at most one half-step, when due. */
void tilt_tick(void);

/* H-5 / F-3.4: drop all four phases now.  Also reached automatically
 * release_ms after the last step. */
void tilt_release(void);

/* Declare the current position to be `deg` without moving.  Used by the
 * zero_tilt flag after the laptop has measured true pitch from the phone. */
void tilt_zero_at(float deg);

void tilt_get_state(tilt_state_t *out);
int32_t tilt_position(void);

/* ---- pure logic, exposed for unit tests ---- */

/* F-3.3 BACKLASH RULE.  The gear train has 1-2 degrees of slop, so every
 * target is approached from the same direction (increasing steps).  If the
 * move would arrive from the other side, overshoot by backlash_steps and come
 * back.  Returns the waypoint to head for first; when it differs from
 * `target`, a second leg follows.
 *
 * from  : current position
 * target: desired position
 * Returns: the first waypoint.  waypoint == target means a single-leg move. */
int32_t tilt_plan_waypoint(int32_t from, int32_t target, int backlash_steps);

/* Angle <-> half-steps.  Rounded, not truncated: at 11.4 steps/degree a
 * truncation would bias every move a tenth of a degree low. */
int32_t tilt_deg_to_steps(float deg, float steps_per_degree);
float tilt_steps_to_deg(int32_t steps, float steps_per_degree);

/* The 8-phase half-step table, exposed so the test can walk it. */
uint8_t tilt_phase_bits(int32_t position);

#ifdef __cplusplus
}
#endif

#endif /* ROVER_TILT_H */
