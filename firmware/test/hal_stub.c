/* hal_stub.c - rover_hal.h for the host test build.
 *
 * Records every call so the tests can assert on pin levels, PWM duty and the
 * stepper phase sequence with no hardware attached.  Time is virtual and
 * advanced explicitly by the test, which is what makes the 1 kHz tilt tick and
 * the 200 ms release timer testable in microseconds of wall clock.
 */

#include "hal_stub.h"

#include <string.h>

#include "protocol.h"

hal_stub_t g_hal;

void hal_stub_reset(void)
{
    memset(&g_hal, 0, sizeof(g_hal));
    for (int i = 0; i < HAL_STUB_MAX_PIN; i++) {
        g_hal.pin_level[i] = -1;   /* -1 = never written */
    }
    g_hal.range_mm = RANGE_INVALID;
    g_hal.vbat_mv = 7400;
    g_hal.range_present = true;
    g_hal.vbat_present = true;
}

void hal_stub_advance_us(int64_t us)
{
    g_hal.now_us += us;
}

void hal_gpio_config_output(int pin)
{
    if (pin >= 0 && pin < HAL_STUB_MAX_PIN) {
        g_hal.pin_configured[pin] = 1;
        g_hal.pin_level[pin] = 0;
    }
}

void hal_gpio_set(int pin, int level)
{
    if (pin >= 0 && pin < HAL_STUB_MAX_PIN) {
        g_hal.pin_level[pin] = level ? 1 : 0;
        g_hal.pin_writes++;
    }
}

void hal_pwm_init(int freq_hz, int timer_bits)
{
    g_hal.pwm_freq = freq_hz;
    g_hal.pwm_bits = timer_bits;
    g_hal.pwm_inited = 1;
}

void hal_pwm_set(int channel, int pin, uint32_t duty)
{
    (void)pin;
    if (channel >= 0 && channel < HAL_STUB_MAX_CHAN) {
        g_hal.pwm_duty[channel] = duty;
        g_hal.pwm_writes++;
    }
}

int64_t hal_now_us(void)
{
    return g_hal.now_us;
}

bool hal_range_init(void) { return g_hal.range_present; }
uint16_t hal_range_read_mm(void) { return g_hal.range_mm; }
bool hal_vbat_init(void) { return g_hal.vbat_present; }
uint16_t hal_vbat_read_mv(void) { return g_hal.vbat_mv; }
void hal_led_set(bool on) { g_hal.led = on ? 1 : 0; }
