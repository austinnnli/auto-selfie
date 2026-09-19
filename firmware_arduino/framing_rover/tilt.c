/* === COPY - DO NOT EDIT ===============================================
 *
 * Generated from firmware/main/tilt.c by tools/gen_arduino_sketch.py.
 * Edit the original and re-run the generator; an edit here is overwritten
 * and tests/test_arduino_sketch.py will fail in the meantime.
 *
 * The copy exists because Arduino IDE only compiles files beside the .ino.
 * ==================================================================== */

/* tilt.c - 28BYJ-48 via ULN2003.
 *
 * H-4 lists three real problems with this motor; two of them are solved here.
 *
 *   Torque   - mechanical.  The mount MUST be balanced on the tilt axis.  The
 *              firmware assumes it is and will not detect slip on its own.
 *   Backlash - 1-2 degrees of slop, handled by tilt_plan_waypoint(): every
 *              target is approached from the same direction.
 *   No home  - there is no homing routine and no limit switch.  The phone's
 *              gravity vector is the absolute reference and the laptop closes
 *              the loop (D-9), which also absorbs slip and missed steps.
 *
 * H-5: half-step, 4096 half-steps per output revolution (0.088 deg/step),
 * maximum reliable rate ~500 half-steps/s, coils de-energised 200 ms after a
 * move.  If the mount droops when released the mount is unbalanced - fix it
 * mechanically, not with holding current.
 */

#include "tilt.h"

#include <math.h>
#include <stdlib.h>

#include "app_config.h"
#include "rover_hal.h"

static tilt_cfg_t s_cfg;
static tilt_state_t s_state;
static int64_t s_next_step_us;
static int64_t s_last_step_us;
static int s_leg;   /* 0 = single or final leg, 1 = overshoot leg pending */

/* H-5: 8-phase half-step table across the four ULN2003 inputs.  Half-stepping
 * rather than full-stepping costs nothing and roughly halves the audible step
 * noise, which matters when the thing is 3 m from someone being photographed. */
static const uint8_t PHASE_TABLE[8] = {
    0x1,  /* 0001 */
    0x3,  /* 0011 */
    0x2,  /* 0010 */
    0x6,  /* 0110 */
    0x4,  /* 0100 */
    0xC,  /* 1100 */
    0x8,  /* 1000 */
    0x9,  /* 1001 */
};

static const int TILT_PINS[4] = {
    PIN_TILT_IN1, PIN_TILT_IN2, PIN_TILT_IN3, PIN_TILT_IN4,
};

/* ------------------------------------------------------------------ */
/* Pure logic - unit tested on the host                               */
/* ------------------------------------------------------------------ */

uint8_t tilt_phase_bits(int32_t position)
{
    /* C's % keeps the sign of the dividend, which would walk the table
     * backwards through index -1 at position -1.  Normalise first. */
    int32_t idx = position % 8;
    if (idx < 0) {
        idx += 8;
    }
    return PHASE_TABLE[idx];
}

int32_t tilt_deg_to_steps(float deg, float steps_per_degree)
{
    return (int32_t)lroundf(deg * steps_per_degree);
}

float tilt_steps_to_deg(int32_t steps, float steps_per_degree)
{
    if (steps_per_degree == 0.0f) {
        return 0.0f;
    }
    return (float)steps / steps_per_degree;
}

int32_t tilt_plan_waypoint(int32_t from, int32_t target, int backlash_steps)
{
    if (backlash_steps <= 0 || target >= from) {
        /* Already approaching from below - the canonical direction. */
        return target;
    }
    /* Moving down: overshoot past the target and come back up, so the gear
     * train is loaded the same way it was when the angle was calibrated. */
    return target - (int32_t)backlash_steps;
}

/* ------------------------------------------------------------------ */
/* Hardware                                                           */
/* ------------------------------------------------------------------ */

static void write_phase(uint8_t bits)
{
    for (int i = 0; i < 4; i++) {
        hal_gpio_set(TILT_PINS[i], (bits >> i) & 1);
    }
}

void tilt_init(const tilt_cfg_t *cfg)
{
    s_cfg = *cfg;
    for (int i = 0; i < 4; i++) {
        hal_gpio_config_output(TILT_PINS[i]);
    }
    s_state.position = 0;
    s_state.target = 0;
    s_state.waypoint = 0;
    s_state.moving = false;
    s_state.energised = false;
    s_leg = 0;
    s_next_step_us = 0;
    s_last_step_us = 0;
    tilt_release();
}

void tilt_set_target_deg(float deg)
{
    if (deg < s_cfg.min_deg) deg = s_cfg.min_deg;
    if (deg > s_cfg.max_deg) deg = s_cfg.max_deg;

    int32_t target = tilt_deg_to_steps(deg, s_cfg.steps_per_degree);
    if (target == s_state.target && s_state.moving) {
        return;   /* already heading there; do not restart the plan */
    }

    s_state.target = target;
    s_state.waypoint = tilt_plan_waypoint(s_state.position, target, s_cfg.backlash_steps);
    s_leg = (s_state.waypoint != target) ? 1 : 0;
    s_state.moving = (s_state.position != s_state.waypoint) || (s_leg == 1);

    if (s_state.moving && s_next_step_us == 0) {
        s_next_step_us = hal_now_us();
    }
}

void tilt_zero_at(float deg)
{
    s_state.position = tilt_deg_to_steps(deg, s_cfg.steps_per_degree);
    s_state.target = s_state.position;
    s_state.waypoint = s_state.position;
    s_state.moving = false;
    s_leg = 0;
}

void tilt_release(void)
{
    write_phase(0x0);
    s_state.energised = false;
}

void tilt_tick(void)
{
    int64_t now = hal_now_us();

    if (!s_state.moving) {
        /* F-3.4 / H-5: de-energise 200 ms after the last step, to avoid heat
         * and a pointless 200 mA sitting on the battery between moves. */
        if (s_state.energised && s_last_step_us != 0 &&
            (now - s_last_step_us) >= (int64_t)s_cfg.release_ms * 1000) {
            tilt_release();
        }
        return;
    }

    if (s_next_step_us != 0 && now < s_next_step_us) {
        return;
    }

    if (s_state.position == s_state.waypoint) {
        if (s_leg == 1) {
            /* Overshoot leg done - turn round and take the target from below. */
            s_state.waypoint = s_state.target;
            s_leg = 0;
            if (s_state.position == s_state.waypoint) {
                s_state.moving = false;
                s_last_step_us = now;
            }
            return;
        }
        s_state.moving = false;
        s_last_step_us = now;
        return;
    }

    int dir = (s_state.waypoint > s_state.position) ? 1 : -1;
    s_state.position += dir;
    write_phase(tilt_phase_bits(s_state.position));
    s_state.energised = true;
    s_last_step_us = now;

    int rate = (s_cfg.max_step_rate > 0) ? s_cfg.max_step_rate : 500;
    s_next_step_us = now + (1000000 / rate);
}

void tilt_get_state(tilt_state_t *out)
{
    *out = s_state;
}

int32_t tilt_position(void)
{
    return s_state.position;
}
