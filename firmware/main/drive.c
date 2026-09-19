/* drive.c - L298N motor drive.
 *
 * F-2.  Input is a signed duty per side, -1000..+1000.  The firmware is
 * deliberately stupid: it applies what it is told, with exactly three
 * transformations, all of which exist for electrical reasons rather than
 * control reasons.
 *
 *   1. trim and min_duty, so a non-zero request actually turns the wheels
 *   2. ramping, to limit current surge
 *   3. direction pins before PWM, never reversing through a live bridge
 *
 * No filtering beyond ramping, no retries, no framing logic (F-5).
 */

#include "drive.h"

#include <stdlib.h>

#include "app_config.h"
#include "rover_hal.h"
#include "protocol.h"

static drive_cfg_t s_cfg;
static drive_state_t s_state;

/* Direction pin pairs, latched so we can tell when a reversal is requested.
 * (DIR_A, DIR_B) = (1,0) forward, (0,1) reverse, (0,0) coast, (1,1) brake. */
static int s_dir_left;   /* -1 reverse, 0 coast, +1 forward */
static int s_dir_right;

/* ------------------------------------------------------------------ */
/* Pure logic - unit tested on the host                               */
/* ------------------------------------------------------------------ */

int drive_apply_min_duty(int duty, int min_duty, int max_duty)
{
    if (duty == 0) {
        return 0;
    }
    int sign = (duty > 0) ? 1 : -1;
    int mag = abs(duty);

    if (mag < min_duty) {
        mag = min_duty;
    }
    if (mag > max_duty) {
        mag = max_duty;
    }
    return sign * mag;
}

int drive_ramp(int current, int target, int max_delta)
{
    if (max_delta <= 0) {
        return target;
    }
    int delta = target - current;
    if (delta > max_delta) {
        return current + max_delta;
    }
    if (delta < -max_delta) {
        return current - max_delta;
    }
    return target;
}

void drive_apply_trim(int left_in, int right_in, float trim, int *left_out, int *right_out)
{
    /* Trim corrects a chassis that pulls to one side on a straight run.  It
     * scales rather than offsets, so it disappears at zero duty and never
     * makes a stopped rover creep. */
    float l = (float)left_in * (1.0f - trim);
    float r = (float)right_in * (1.0f + trim);
    *left_out = proto_clamp_duty((int32_t)l);
    *right_out = proto_clamp_duty((int32_t)r);
}

/* ------------------------------------------------------------------ */
/* Hardware application                                               */
/* ------------------------------------------------------------------ */

static void set_direction_pins(int pin_a, int pin_b, int dir)
{
    switch (dir) {
    case 1:   /* forward */
        hal_gpio_set(pin_a, 1);
        hal_gpio_set(pin_b, 0);
        break;
    case -1:  /* reverse */
        hal_gpio_set(pin_a, 0);
        hal_gpio_set(pin_b, 1);
        break;
    default:  /* coast - both low.  Never (1,1): braking an L298N into a
               * stalled gearbox is a current spike with no upside here. */
        hal_gpio_set(pin_a, 0);
        hal_gpio_set(pin_b, 0);
        break;
    }
}

static uint32_t duty_to_ledc(int magnitude)
{
    /* -1000..1000 on the wire, 0..1023 in the timer. */
    if (magnitude <= 0) {
        return 0;
    }
    if (magnitude >= DUTY_MAX) {
        return (uint32_t)LEDC_MAX_DUTY;
    }
    return (uint32_t)((magnitude * LEDC_MAX_DUTY) / DUTY_MAX);
}

/* F-2.3: set direction pins first, then PWM, and never change direction
 * without passing through zero duty.  A reversal therefore costs one tick. */
static void apply_side(int pin_pwm, int channel, int pin_a, int pin_b,
                       int *cached_dir, int duty)
{
    int want_dir = (duty > 0) ? 1 : (duty < 0) ? -1 : 0;

    if (want_dir != *cached_dir && *cached_dir != 0 && want_dir != 0) {
        /* Reversal requested while the bridge is live: kill PWM, drop the
         * direction pins, and let the next tick bring the duty up in the new
         * direction. */
        hal_pwm_set(channel, pin_pwm, 0);
        set_direction_pins(pin_a, pin_b, 0);
        *cached_dir = 0;
        return;
    }

    if (want_dir != *cached_dir) {
        set_direction_pins(pin_a, pin_b, want_dir);
        *cached_dir = want_dir;
    }
    hal_pwm_set(channel, pin_pwm, duty_to_ledc(abs(duty)));
}

/* ------------------------------------------------------------------ */
/* Public interface                                                   */
/* ------------------------------------------------------------------ */

void drive_init(const drive_cfg_t *cfg)
{
    s_cfg = *cfg;

    hal_gpio_config_output(PIN_DIR_L_A);
    hal_gpio_config_output(PIN_DIR_L_B);
    hal_gpio_config_output(PIN_DIR_R_A);
    hal_gpio_config_output(PIN_DIR_R_B);

    hal_pwm_init(s_cfg.pwm_freq, LEDC_TIMER_BITS);

    s_state.applied_left = 0;
    s_state.applied_right = 0;
    s_state.target_left = 0;
    s_state.target_right = 0;
    s_state.coasting = true;
    s_dir_left = 0;
    s_dir_right = 0;

    drive_coast();
}

void drive_set(int left, int right)
{
    s_state.target_left = proto_clamp_duty(left);
    s_state.target_right = proto_clamp_duty(right);
    s_state.coasting = false;
}

void drive_coast(void)
{
    /* F-2.5.  Immediate, and it also clears the ramp state so re-enabling
     * starts from a standstill rather than snapping back to the old duty. */
    s_state.target_left = 0;
    s_state.target_right = 0;
    s_state.applied_left = 0;
    s_state.applied_right = 0;
    s_state.coasting = true;
    s_dir_left = 0;
    s_dir_right = 0;

    hal_pwm_set(LEDC_CHANNEL_LEFT, PIN_PWM_L, 0);
    hal_pwm_set(LEDC_CHANNEL_RIGHT, PIN_PWM_R, 0);
    set_direction_pins(PIN_DIR_L_A, PIN_DIR_L_B, 0);
    set_direction_pins(PIN_DIR_R_A, PIN_DIR_R_B, 0);
}

void drive_tick(void)
{
    if (s_state.coasting) {
        return;
    }

    int trimmed_l, trimmed_r;
    drive_apply_trim(s_state.target_left, s_state.target_right, s_cfg.trim,
                     &trimmed_l, &trimmed_r);

    int want_l = drive_apply_min_duty(trimmed_l, s_cfg.min_duty_left, s_cfg.max_duty);
    int want_r = drive_apply_min_duty(trimmed_r, s_cfg.min_duty_right, s_cfg.max_duty);

    /* Ramp toward the request rather than stepping to it (F-2.2).  Crossing
     * zero is allowed here; apply_side() is what refuses to reverse a live
     * bridge, and it costs one tick to pass through zero. */
    s_state.applied_left = drive_ramp(s_state.applied_left, want_l, s_cfg.max_duty_per_tick);
    s_state.applied_right = drive_ramp(s_state.applied_right, want_r, s_cfg.max_duty_per_tick);

    /* Ramping through the dead band would leave the bridge energised but the
     * wheels stalled; snap across it instead. */
    if (want_l == 0 && abs(s_state.applied_left) < s_cfg.min_duty_left) {
        s_state.applied_left = 0;
    }
    if (want_r == 0 && abs(s_state.applied_right) < s_cfg.min_duty_right) {
        s_state.applied_right = 0;
    }

    apply_side(PIN_PWM_L, LEDC_CHANNEL_LEFT, PIN_DIR_L_A, PIN_DIR_L_B,
               &s_dir_left, s_state.applied_left);
    apply_side(PIN_PWM_R, LEDC_CHANNEL_RIGHT, PIN_DIR_R_A, PIN_DIR_R_B,
               &s_dir_right, s_state.applied_right);
}

void drive_get_state(drive_state_t *out)
{
    *out = s_state;
}
