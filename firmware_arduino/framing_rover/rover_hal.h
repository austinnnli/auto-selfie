/* === COPY - DO NOT EDIT ===============================================
 *
 * Generated from firmware/main/rover_hal.h by tools/gen_arduino_sketch.py.
 * Edit the original and re-run the generator; an edit here is overwritten
 * and tests/test_arduino_sketch.py will fail in the meantime.
 *
 * The copy exists because Arduino IDE only compiles files beside the .ino.
 * ==================================================================== */

/* rover_hal.h - the thin line between logic and silicon.
 *
 * drive.c, tilt.c and safety.c talk to the hardware only through the handful
 * of calls below.  On the ESP32-S3 they are inline wrappers over ESP-IDF.  On
 * a host build (-DROVER_HOST_TEST) they are implemented by test/hal_stub.c,
 * which records every call so the unit tests can assert on pin states, duty
 * values and step sequences without a rover on the desk.
 *
 * This is the whole reason the firmware's tricky parts - min-duty
 * compensation, ramping, backlash compensation - can be tested at a desk.
 * Keep it small: if a module needs a new ESP-IDF call, add it here rather
 * than including driver/ headers directly, or the host tests stop building.
 */

#ifndef ROVER_HAL_H
#define ROVER_HAL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- lifecycle ---------------------------------------------------- */
void hal_gpio_config_output(int pin);
void hal_gpio_set(int pin, int level);

/* PWM.  channel is LEDC_CHANNEL_LEFT / LEDC_CHANNEL_RIGHT; duty is
 * 0..LEDC_MAX_DUTY (10-bit). */
void hal_pwm_init(int freq_hz, int timer_bits);
void hal_pwm_set(int channel, int pin, uint32_t duty);

/* Monotonic microseconds since boot. */
int64_t hal_now_us(void);
static inline int64_t hal_now_ms(void) { return hal_now_us() / 1000; }

/* Range sensor.  Returns RANGE_INVALID (0xFFFF) when there is no sensor or
 * no fresh reading; safety.c treats that as "unknown", never as "clear". */
bool hal_range_init(void);
uint16_t hal_range_read_mm(void);

/* Battery.  Millivolts at the pack, after the divider correction. */
bool hal_vbat_init(void);
uint16_t hal_vbat_read_mv(void);

/* Status LED; pin < 0 means the board has none. */
void hal_led_set(bool on);

#ifdef __cplusplus
}
#endif

#endif /* ROVER_HAL_H */
