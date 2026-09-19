/* safety.c - the three checks that run regardless of what the laptop asks.
 *
 * F-4:
 *   - no valid packet in watchdog_ms, or a sequence that has not advanced,
 *     forces duty 0
 *   - distance below range_stop_mm refuses positive (forward) duty, while
 *     still allowing reverse and turns
 *   - supply below vbat_min_mv refuses all motion and raises a status bit
 *
 * All three are enforced here, in the 100 Hz safety task on core 1, so they
 * keep working when Wi-Fi is gone, the laptop is off, or the net task is
 * blocked.  That is the whole point of design rule 4.
 */

#include "safety.h"

#include "app_config.h"
#include "drive.h"
#include "protocol.h"
#include "rover_hal.h"

static safety_cfg_t s_cfg;
static safety_status_t s_status;
static int64_t s_last_feed_ms;
static bool s_range_present;
static bool s_vbat_present;

void safety_init(const safety_cfg_t *cfg)
{
    s_cfg = *cfg;
    s_status.watchdog_trip = true;   /* tripped until the first packet arrives */
    s_status.range_trip = false;
    s_status.vbat_low = false;
    s_status.range_mm = RANGE_INVALID;
    s_status.vbat_mv = 0;
    s_last_feed_ms = 0;

    s_range_present = hal_range_init();
    s_vbat_present = hal_vbat_init();
}

void safety_feed(int64_t now_ms)
{
    s_last_feed_ms = now_ms;
    s_status.watchdog_trip = false;
}

void safety_limit_forward(int left_in, int right_in, int *left_out, int *right_out)
{
    int common = (left_in + right_in) / 2;
    int diff = (left_in - right_in) / 2;

    if (common > 0) {
        common = 0;
    }
    *left_out = proto_clamp_duty(common + diff);
    *right_out = proto_clamp_duty(common - diff);
}

bool safety_motion_allowed(bool enabled, const safety_status_t *st)
{
    if (!enabled) {
        return false;
    }
    if (st->watchdog_trip) {
        return false;
    }
    if (st->vbat_low) {
        return false;
    }
    return true;
}

void safety_tick(void)
{
    int64_t now = hal_now_ms();

    /* Watchdog.  s_last_feed_ms == 0 means nothing has ever arrived. */
    if (s_last_feed_ms == 0 || (now - s_last_feed_ms) > (int64_t)s_cfg.watchdog_ms) {
        if (!s_status.watchdog_trip) {
            s_status.watchdog_trip = true;
        }
        drive_coast();
    }

    /* Range.  An unreadable sensor is "unknown", never "clear": a sensor that
     * has failed open must not silently re-enable forward motion. */
    if (s_range_present) {
        uint16_t mm = hal_range_read_mm();
        s_status.range_mm = mm;
        if (mm != RANGE_INVALID) {
            s_status.range_trip = (mm < (uint16_t)s_cfg.range_stop_mm);
        }
        /* mm == RANGE_INVALID: hold the previous trip state rather than
         * clearing it.  A dropped reading is not evidence of clear ground. */
    } else {
        s_status.range_mm = RANGE_INVALID;
    }

    /* Battery.  Below vbat_min_mv all motion is refused - a browning-out
     * bridge is how you lose a gearbox. */
    if (s_vbat_present) {
        uint16_t mv = hal_vbat_read_mv();
        s_status.vbat_mv = mv;
        if (mv > 0) {
            s_status.vbat_low = (mv < (uint16_t)s_cfg.vbat_min_mv);
        }
    }

    if (s_status.vbat_low) {
        drive_coast();
    }
}

void safety_get(safety_status_t *out)
{
    *out = s_status;
}

uint8_t safety_status_bits(bool enabled, bool tilt_moving)
{
    uint8_t bits = 0;
    if (enabled) bits |= ST_ENABLED;
    if (s_status.range_trip) bits |= ST_RANGE_TRIP;
    if (s_status.vbat_low) bits |= ST_VBAT_LOW;
    if (s_status.watchdog_trip) bits |= ST_WATCHDOG_TRIP;
    if (tilt_moving) bits |= ST_TILT_MOVING;
    return bits;
}
