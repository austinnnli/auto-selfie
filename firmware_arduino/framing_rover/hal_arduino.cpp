/* hal_arduino.cpp - rover_hal.h implemented against the Arduino-ESP32 core.
 *
 * The ESP-IDF build uses firmware/main/hal_esp32.c instead; nothing else in
 * the firmware differs between the two.  That is the whole point of keeping
 * the hardware behind ten functions.
 *
 * Arduino-ESP32 2.x and 3.x have incompatible LEDC and ADC APIs, so both are
 * handled below.  3.x is the one to prefer: its analogReadMilliVolts() uses
 * the chip's factory eFuse calibration, which is worth 50-100 mV on the
 * battery reading and therefore worth a real margin on vbat_min_mv.
 */

#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>

extern "C" {
#include "app_config.h"
#include "protocol.h"
#include "rover_hal.h"
}

static bool s_i2c_ok = false;

/* ------------------------------------------------------------------ */
/* GPIO                                                               */
/* ------------------------------------------------------------------ */

extern "C" void hal_gpio_config_output(int pin)
{
    if (pin < 0) {
        return;
    }
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

extern "C" void hal_gpio_set(int pin, int level)
{
    if (pin < 0) {
        return;
    }
    digitalWrite(pin, level ? HIGH : LOW);
}

/* ------------------------------------------------------------------ */
/* PWM                                                                */
/* H-3: 1 kHz on ENA/ENB.  Do NOT raise this to 20 kHz - an L298N is a  */
/* bipolar bridge, not a MOSFET one, and its switching losses at        */
/* ultrasonic rates are what cooks the heatsink.                        */
/* ------------------------------------------------------------------ */

extern "C" void hal_pwm_init(int freq_hz, int timer_bits)
{
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(PIN_PWM_L, freq_hz, timer_bits);
    ledcAttach(PIN_PWM_R, freq_hz, timer_bits);
#else
    ledcSetup(LEDC_CHANNEL_LEFT, freq_hz, timer_bits);
    ledcSetup(LEDC_CHANNEL_RIGHT, freq_hz, timer_bits);
    ledcAttachPin(PIN_PWM_L, LEDC_CHANNEL_LEFT);
    ledcAttachPin(PIN_PWM_R, LEDC_CHANNEL_RIGHT);
#endif
    log_i("ledc at %d Hz, %d-bit (H-3: do NOT raise this to 20 kHz)",
          freq_hz, timer_bits);
}

extern "C" void hal_pwm_set(int channel, int pin, uint32_t duty)
{
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    /* 3.x addresses the channel by the pin it is attached to. */
    (void)channel;
    ledcWrite(pin, duty);
#else
    (void)pin;
    ledcWrite(channel, duty);
#endif
}

/* ------------------------------------------------------------------ */
/* time                                                               */
/* ------------------------------------------------------------------ */

extern "C" int64_t hal_now_us(void)
{
    /* esp_timer_get_time(), not micros(): micros() is 32-bit and wraps every
     * 71 minutes.  The tilt task compares absolute microsecond deadlines, so a
     * wrap would freeze the stepper until the counter came round again - a
     * fault that only appears after an hour of running, which is the worst
     * kind to debug in a field. */
    return esp_timer_get_time();
}

/* ------------------------------------------------------------------ */
/* range sensor                                                       */
/* ------------------------------------------------------------------ */

extern "C" bool hal_range_init(void)
{
    Wire.begin(PIN_RANGE_SDA, PIN_RANGE_SCL, 100000);
    Wire.beginTransmission(VL53L1X_ADDR);
    s_i2c_ok = (Wire.endTransmission() == 0);

    if (!s_i2c_ok) {
        log_w("no device at 0x%02X on sda=%d scl=%d - range stop is inactive",
              VL53L1X_ADDR, PIN_RANGE_SDA, PIN_RANGE_SCL);
        return false;
    }
    /* Bring-up step 9 (F-6) is where the vendor driver goes.  Install the
     * "SparkFun VL53L1X 4m Laser Distance Sensor" library, then call its
     * begin()/startRanging() here and its getDistance() below.  Until then
     * the sensor honestly reports "no reading". */
    log_i("VL53L1X present; add the vendor driver at F-6 step 9");
    return true;
}

extern "C" uint16_t hal_range_read_mm(void)
{
    if (!s_i2c_ok) {
        return RANGE_INVALID;
    }
    /* Vendor driver call goes here.  RANGE_INVALID means "unknown", and
     * safety.c never reads that as "clear" - it holds the previous trip
     * state rather than re-enabling forward motion on a failed sensor. */
    return RANGE_INVALID;
}

/* ------------------------------------------------------------------ */
/* battery                                                            */
/* ------------------------------------------------------------------ */

extern "C" bool hal_vbat_init(void)
{
    analogReadResolution(12);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    analogSetPinAttenuation(PIN_VBAT_ADC, ADC_11db);
#else
    analogSetPinAttenuation(PIN_VBAT_ADC, ADC_11db);
#endif
    return true;
}

extern "C" uint16_t hal_vbat_read_mv(void)
{
    /* analogReadMilliVolts() applies the chip's factory eFuse calibration,
     * which raw analogRead() does not.  VBAT_DIVIDER_* then undoes the
     * potential divider - set those from a multimeter reading, not from the
     * nominal resistor values (CF-2). */
    uint32_t mv = analogReadMilliVolts(PIN_VBAT_ADC);
    return (uint16_t)((mv * VBAT_DIVIDER_NUM) / VBAT_DIVIDER_DEN);
}

extern "C" void hal_led_set(bool on)
{
    if (PIN_STATUS_LED < 0) {
        return;
    }
    digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}
