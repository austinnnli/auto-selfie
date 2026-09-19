/* test_drive_tilt.c - the firmware logic that is worth testing at a desk.
 *
 * Unit tests named in the PRD's test plan:
 *   - apply_min_duty and rate_limit edge cases
 *   - backlash compensation reaching the same angle from both directions
 *
 * Plus the things that bite during bring-up: never reversing a live bridge,
 * the stepper phase table not walking backwards through a negative index, and
 * the range trip leaving turns available while refusing forward motion.
 */

#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "drive.h"
#include "hal_stub.h"
#include "protocol.h"
#include "safety.h"
#include "test_util.h"
#include "tilt.h"

int g_tests_run = 0;
int g_tests_failed = 0;
const char *g_current_test = NULL;

static drive_cfg_t default_drive_cfg(void)
{
    drive_cfg_t c = {
        .min_duty_left = CFG_MIN_DUTY_LEFT,
        .min_duty_right = CFG_MIN_DUTY_RIGHT,
        .max_duty = CFG_MAX_DUTY,
        .max_duty_per_tick = CFG_MAX_DUTY_PER_TICK,
        .trim = 0.0f,                 /* trim has its own test */
        .pwm_freq = CFG_PWM_FREQ,
    };
    return c;
}

static tilt_cfg_t default_tilt_cfg(void)
{
    tilt_cfg_t c = {
        .steps_per_degree = CFG_STEPS_PER_DEGREE,
        .backlash_steps = CFG_BACKLASH_STEPS,
        .max_step_rate = CFG_MAX_STEP_RATE,
        .release_ms = CFG_TILT_RELEASE_MS,
        .min_deg = CFG_TILT_MIN_DEG,
        .max_deg = CFG_TILT_MAX_DEG,
    };
    return c;
}

/* ------------------------------------------------------------------ */
/* min duty                                                           */
/* ------------------------------------------------------------------ */

static void test_min_duty_zero_stays_zero(void)
{
    /* A zero request must stay zero.  Lifting it to min_duty would make the
     * rover creep whenever the control law is satisfied - the single most
     * embarrassing possible bug in this system. */
    CHECK_EQ_INT(drive_apply_min_duty(0, 320, 650), 0);
}

static void test_min_duty_lifts_small_requests(void)
{
    CHECK_EQ_INT(drive_apply_min_duty(1, 320, 650), 320);
    CHECK_EQ_INT(drive_apply_min_duty(-1, 320, 650), -320);
    CHECK_EQ_INT(drive_apply_min_duty(319, 320, 650), 320);
    CHECK_EQ_INT(drive_apply_min_duty(-319, 320, 650), -320);
}

static void test_min_duty_is_idempotent(void)
{
    /* The laptop's decide.py maps its normalised command onto the same live
     * band.  Applying the floor twice must not change anything, or the two
     * layers would fight. */
    for (int d = -1000; d <= 1000; d += 7) {
        int once = drive_apply_min_duty(d, 320, 650);
        int twice = drive_apply_min_duty(once, 320, 650);
        CHECK_EQ_INT(twice, once);
    }
}

static void test_min_duty_clamps_to_max(void)
{
    CHECK_EQ_INT(drive_apply_min_duty(1000, 320, 650), 650);
    CHECK_EQ_INT(drive_apply_min_duty(-1000, 320, 650), -650);
}

static void test_min_duty_survives_min_above_max(void)
{
    /* A miskeyed config must not produce a sign flip or a wild value. */
    CHECK_EQ_INT(drive_apply_min_duty(100, 700, 650), 650);
    CHECK_EQ_INT(drive_apply_min_duty(-100, 700, 650), -650);
}

/* ------------------------------------------------------------------ */
/* ramping                                                            */
/* ------------------------------------------------------------------ */

static void test_ramp_limits_step(void)
{
    CHECK_EQ_INT(drive_ramp(0, 1000, 40), 40);
    CHECK_EQ_INT(drive_ramp(0, -1000, 40), -40);
    CHECK_EQ_INT(drive_ramp(400, 420, 40), 420);     /* inside the limit */
    CHECK_EQ_INT(drive_ramp(400, 300, 40), 360);
}

static void test_ramp_reaches_target_exactly(void)
{
    int cur = 0;
    for (int i = 0; i < 100; i++) {
        cur = drive_ramp(cur, 650, 40);
    }
    CHECK_EQ_INT(cur, 650);   /* not 640, not 680 */
}

static void test_ramp_zero_delta_is_immediate(void)
{
    /* max_duty_per_tick <= 0 means "no limit", used by the bench sender. */
    CHECK_EQ_INT(drive_ramp(0, 650, 0), 650);
    CHECK_EQ_INT(drive_ramp(0, 650, -1), 650);
}

/* ------------------------------------------------------------------ */
/* trim                                                               */
/* ------------------------------------------------------------------ */

static void test_trim_is_zero_at_standstill(void)
{
    int l, r;
    drive_apply_trim(0, 0, -0.03f, &l, &r);
    CHECK_EQ_INT(l, 0);
    CHECK_EQ_INT(r, 0);   /* a stopped rover must never creep */
}

static void test_trim_biases_the_right_way(void)
{
    int l, r;
    /* Positive trim slows the left side, steering the rover right. */
    drive_apply_trim(500, 500, 0.10f, &l, &r);
    CHECK_EQ_INT(l, 450);
    CHECK_EQ_INT(r, 550);
}

/* ------------------------------------------------------------------ */
/* direction handling                                                 */
/* ------------------------------------------------------------------ */

static void test_coast_drops_all_pins(void)
{
    hal_stub_reset();
    drive_cfg_t cfg = default_drive_cfg();
    drive_init(&cfg);
    drive_set(600, 600);
    for (int i = 0; i < 50; i++) drive_tick();

    drive_coast();
    CHECK_EQ_INT(g_hal.pin_level[PIN_DIR_L_A], 0);
    CHECK_EQ_INT(g_hal.pin_level[PIN_DIR_L_B], 0);
    CHECK_EQ_INT(g_hal.pin_level[PIN_DIR_R_A], 0);
    CHECK_EQ_INT(g_hal.pin_level[PIN_DIR_R_B], 0);
    CHECK_EQ_INT(g_hal.pwm_duty[LEDC_CHANNEL_LEFT], 0);
    CHECK_EQ_INT(g_hal.pwm_duty[LEDC_CHANNEL_RIGHT], 0);
}

static void test_never_reverses_a_live_bridge(void)
{
    hal_stub_reset();
    drive_cfg_t cfg = default_drive_cfg();
    drive_init(&cfg);

    drive_set(600, 600);
    for (int i = 0; i < 50; i++) drive_tick();
    CHECK_EQ_INT(g_hal.pin_level[PIN_DIR_L_A], 1);
    CHECK_EQ_INT(g_hal.pin_level[PIN_DIR_L_B], 0);
    CHECK(g_hal.pwm_duty[LEDC_CHANNEL_LEFT] > 0);

    /* Ask for full reverse.  F-2.3: the duty must reach zero and the
     * direction pins must go low before the other direction is asserted. */
    drive_set(-600, -600);
    bool saw_zero_before_reverse = false;
    for (int i = 0; i < 80; i++) {
        drive_tick();
        int a = g_hal.pin_level[PIN_DIR_L_A];
        int b = g_hal.pin_level[PIN_DIR_L_B];
        uint32_t duty = g_hal.pwm_duty[LEDC_CHANNEL_LEFT];
        if (b == 1) {
            /* Reverse is now asserted; we must have passed through a moment
             * with both pins low. */
            CHECK(saw_zero_before_reverse);
            break;
        }
        if (a == 0 && b == 0 && duty == 0) {
            saw_zero_before_reverse = true;
        }
    }
    CHECK(saw_zero_before_reverse);
    CHECK_EQ_INT(g_hal.pin_level[PIN_DIR_L_B], 1);
}

static void test_ramp_does_not_park_in_the_deadband(void)
{
    /* Coming back to zero, the ramp passes through duties the bridge cannot
     * turn.  Sitting there would heat the motors for no motion, so the last
     * part of the ramp snaps across. */
    hal_stub_reset();
    drive_cfg_t cfg = default_drive_cfg();
    drive_init(&cfg);
    drive_set(600, 600);
    for (int i = 0; i < 50; i++) drive_tick();

    drive_set(0, 0);
    for (int i = 0; i < 50; i++) drive_tick();

    drive_state_t st;
    drive_get_state(&st);
    CHECK_EQ_INT(st.applied_left, 0);
    CHECK_EQ_INT(st.applied_right, 0);
}

/* ------------------------------------------------------------------ */
/* tilt: phase table and backlash                                     */
/* ------------------------------------------------------------------ */

static void test_phase_table_is_a_valid_half_step_sequence(void)
{
    /* Each step changes exactly one coil - that is what makes it a half-step
     * sequence rather than a full-step one with a typo. */
    for (int i = 0; i < 8; i++) {
        uint8_t a = tilt_phase_bits(i);
        uint8_t b = tilt_phase_bits(i + 1);
        int diff = a ^ b;
        int bits = 0;
        for (int k = 0; k < 4; k++) bits += (diff >> k) & 1;
        CHECK_EQ_INT(bits, 1);
    }
}

static void test_phase_table_handles_negative_positions(void)
{
    /* C's % keeps the sign of the dividend.  Without normalising, position -1
     * would index PHASE_TABLE[-1] and read whatever is in front of the array. */
    CHECK_EQ_INT(tilt_phase_bits(-1), tilt_phase_bits(7));
    CHECK_EQ_INT(tilt_phase_bits(-8), tilt_phase_bits(0));
    CHECK_EQ_INT(tilt_phase_bits(-9), tilt_phase_bits(7));
}

static void test_waypoint_upward_move_is_single_leg(void)
{
    /* Increasing steps is the canonical approach direction; no overshoot. */
    CHECK_EQ_INT(tilt_plan_waypoint(0, 100, 24), 100);
    CHECK_EQ_INT(tilt_plan_waypoint(-500, -100, 24), -100);
}

static void test_waypoint_downward_move_overshoots(void)
{
    CHECK_EQ_INT(tilt_plan_waypoint(100, 0, 24), -24);
    CHECK_EQ_INT(tilt_plan_waypoint(0, -100, 24), -124);
}

static void test_waypoint_no_backlash_configured(void)
{
    CHECK_EQ_INT(tilt_plan_waypoint(100, 0, 0), 0);
}

static void run_tilt_until_idle(int max_ticks)
{
    for (int i = 0; i < max_ticks; i++) {
        tilt_tick();
        hal_stub_advance_us(1000);
        tilt_state_t st;
        tilt_get_state(&st);
        if (!st.moving) {
            return;
        }
    }
}

static void test_same_angle_from_both_directions(void)
{
    /* THE backlash test named in the PRD: reach 10 degrees from below and from
     * above, and land on the same step position. */
    hal_stub_reset();
    tilt_cfg_t cfg = default_tilt_cfg();

    tilt_init(&cfg);
    tilt_zero_at(0.0f);
    tilt_set_target_deg(10.0f);
    run_tilt_until_idle(20000);
    int32_t from_below = tilt_position();

    tilt_init(&cfg);
    tilt_zero_at(25.0f);
    tilt_set_target_deg(10.0f);
    run_tilt_until_idle(20000);
    int32_t from_above = tilt_position();

    CHECK_EQ_INT(from_below, from_above);
    CHECK_EQ_INT(from_below, tilt_deg_to_steps(10.0f, cfg.steps_per_degree));
}

static void test_downward_move_actually_overshoots(void)
{
    /* Not just the endpoint: the path must dip below the target and come back,
     * or the gear slop is still in the wrong place. */
    hal_stub_reset();
    tilt_cfg_t cfg = default_tilt_cfg();
    tilt_init(&cfg);
    tilt_zero_at(20.0f);

    int32_t target = tilt_deg_to_steps(5.0f, cfg.steps_per_degree);
    tilt_set_target_deg(5.0f);

    int32_t lowest = tilt_position();
    for (int i = 0; i < 20000; i++) {
        tilt_tick();
        hal_stub_advance_us(1000);
        if (tilt_position() < lowest) lowest = tilt_position();
        tilt_state_t st;
        tilt_get_state(&st);
        if (!st.moving) break;
    }
    CHECK(lowest <= target - cfg.backlash_steps);
    CHECK_EQ_INT(tilt_position(), target);
}

static void test_tilt_respects_step_rate(void)
{
    /* H-5: 500 half-steps/s maximum.  Ticking at 1 kHz must not produce
     * 1000 steps/s. */
    hal_stub_reset();
    tilt_cfg_t cfg = default_tilt_cfg();
    tilt_init(&cfg);
    tilt_zero_at(0.0f);
    tilt_set_target_deg(30.0f);

    for (int i = 0; i < 1000; i++) {   /* exactly one second of ticks */
        tilt_tick();
        hal_stub_advance_us(1000);
    }
    int32_t moved = tilt_position();
    CHECK(moved <= cfg.max_step_rate + 1);
    CHECK(moved > cfg.max_step_rate / 2);
}

static void test_tilt_de_energises_after_release_ms(void)
{
    /* H-5: coils off 200 ms after a move, or the motor cooks and the pack
     * drains between shots. */
    hal_stub_reset();
    tilt_cfg_t cfg = default_tilt_cfg();
    tilt_init(&cfg);
    tilt_zero_at(0.0f);
    tilt_set_target_deg(1.0f);
    run_tilt_until_idle(5000);

    tilt_state_t st;
    tilt_get_state(&st);
    CHECK(st.energised);

    for (int i = 0; i < 250; i++) {
        hal_stub_advance_us(1000);
        tilt_tick();
    }
    tilt_get_state(&st);
    CHECK(!st.energised);
    CHECK_EQ_INT(g_hal.pin_level[PIN_TILT_IN1], 0);
    CHECK_EQ_INT(g_hal.pin_level[PIN_TILT_IN2], 0);
    CHECK_EQ_INT(g_hal.pin_level[PIN_TILT_IN3], 0);
    CHECK_EQ_INT(g_hal.pin_level[PIN_TILT_IN4], 0);
}

static void test_tilt_clamps_to_mechanical_range(void)
{
    hal_stub_reset();
    tilt_cfg_t cfg = default_tilt_cfg();
    tilt_init(&cfg);
    tilt_zero_at(0.0f);
    tilt_set_target_deg(400.0f);
    run_tilt_until_idle(60000);
    CHECK_EQ_INT(tilt_position(), tilt_deg_to_steps(cfg.max_deg, cfg.steps_per_degree));
}

static void test_deg_step_round_trip(void)
{
    tilt_cfg_t cfg = default_tilt_cfg();
    for (float d = -30.0f; d <= 30.0f; d += 0.5f) {
        int32_t s = tilt_deg_to_steps(d, cfg.steps_per_degree);
        float back = tilt_steps_to_deg(s, cfg.steps_per_degree);
        float err = back - d;
        if (err < 0) err = -err;
        CHECK(err < 0.05f);   /* half a step is 0.044 degrees */
    }
}

/* ------------------------------------------------------------------ */
/* safety                                                             */
/* ------------------------------------------------------------------ */

static void test_range_trip_refuses_forward_allows_reverse(void)
{
    int l, r;
    safety_limit_forward(600, 600, &l, &r);
    CHECK_EQ_INT(l, 0);
    CHECK_EQ_INT(r, 0);

    safety_limit_forward(-600, -600, &l, &r);
    CHECK_EQ_INT(l, -600);
    CHECK_EQ_INT(r, -600);
}

static void test_range_trip_still_allows_a_spin(void)
{
    /* Clamping each side independently would turn a spin into a one-sided arc.
     * The turning component must survive untouched. */
    int l, r;
    safety_limit_forward(500, -500, &l, &r);
    CHECK_EQ_INT(l, 500);
    CHECK_EQ_INT(r, -500);

    /* A forward arc loses only its forward part. */
    safety_limit_forward(600, 200, &l, &r);
    CHECK_EQ_INT(l, 200);
    CHECK_EQ_INT(r, -200);
}

static void test_motion_gate(void)
{
    safety_status_t st = {0};
    CHECK(safety_motion_allowed(true, &st));
    CHECK(!safety_motion_allowed(false, &st));

    st.watchdog_trip = true;
    CHECK(!safety_motion_allowed(true, &st));

    st.watchdog_trip = false;
    st.vbat_low = true;
    CHECK(!safety_motion_allowed(true, &st));

    /* A range trip alone does NOT forbid motion - it only removes forward. */
    st.vbat_low = false;
    st.range_trip = true;
    CHECK(safety_motion_allowed(true, &st));
}

int main(void)
{
    printf("drive / tilt / safety (host build)\n");
    RUN(test_min_duty_zero_stays_zero);
    RUN(test_min_duty_lifts_small_requests);
    RUN(test_min_duty_is_idempotent);
    RUN(test_min_duty_clamps_to_max);
    RUN(test_min_duty_survives_min_above_max);
    RUN(test_ramp_limits_step);
    RUN(test_ramp_reaches_target_exactly);
    RUN(test_ramp_zero_delta_is_immediate);
    RUN(test_trim_is_zero_at_standstill);
    RUN(test_trim_biases_the_right_way);
    RUN(test_coast_drops_all_pins);
    RUN(test_never_reverses_a_live_bridge);
    RUN(test_ramp_does_not_park_in_the_deadband);
    RUN(test_phase_table_is_a_valid_half_step_sequence);
    RUN(test_phase_table_handles_negative_positions);
    RUN(test_waypoint_upward_move_is_single_leg);
    RUN(test_waypoint_downward_move_overshoots);
    RUN(test_waypoint_no_backlash_configured);
    RUN(test_same_angle_from_both_directions);
    RUN(test_downward_move_actually_overshoots);
    RUN(test_tilt_respects_step_rate);
    RUN(test_tilt_de_energises_after_release_ms);
    RUN(test_tilt_clamps_to_mechanical_range);
    RUN(test_deg_step_round_trip);
    RUN(test_range_trip_refuses_forward_allows_reverse);
    RUN(test_range_trip_still_allows_a_spin);
    RUN(test_motion_gate);
    TEST_MAIN_EPILOGUE();
}
