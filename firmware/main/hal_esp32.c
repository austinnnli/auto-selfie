/* hal_esp32.c - rover_hal.h implemented against ESP-IDF.
 *
 * The host test build (-DROVER_HOST_TEST) swaps this file for
 * test/hal_stub.c; nothing else in the firmware changes.
 */

#ifndef ROVER_HOST_TEST

#include "rover_hal.h"

#include "app_config.h"
#include "protocol.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "hal";
static adc_oneshot_unit_handle_t s_adc;
static bool s_adc_ok;
static bool s_i2c_ok;

void hal_gpio_config_output(int pin)
{
    if (pin < 0) {
        return;
    }
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(pin, 0);
}

void hal_gpio_set(int pin, int level)
{
    if (pin < 0) {
        return;
    }
    gpio_set_level(pin, level);
}

void hal_pwm_init(int freq_hz, int timer_bits)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = (ledc_timer_bit_t)timer_bits,
        .freq_hz = freq_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    const int pins[2] = {PIN_PWM_L, PIN_PWM_R};
    const int chans[2] = {LEDC_CHANNEL_LEFT, LEDC_CHANNEL_RIGHT};
    for (int i = 0; i < 2; i++) {
        ledc_channel_config_t ch = {
            .gpio_num = pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)chans[i],
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch));
    }
    ESP_LOGI(TAG, "ledc at %d Hz, %d-bit (H-3: do NOT raise this to 20 kHz)",
             freq_hz, timer_bits);
}

void hal_pwm_set(int channel, int pin, uint32_t duty)
{
    (void)pin;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel);
}

int64_t hal_now_us(void)
{
    return esp_timer_get_time();
}

/* ---- VL53L1X ------------------------------------------------------ */

bool hal_range_init(void)
{
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_RANGE_SDA,
        .scl_io_num = PIN_RANGE_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    if (i2c_param_config(I2C_NUM_0, &cfg) != ESP_OK) {
        return false;
    }
    if (i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0) != ESP_OK) {
        return false;
    }
    /* A full VL53L1X init sequence lives in the vendor driver; bring-up step 9
     * (F-6) is where you drop it in.  Until then the sensor reports "no
     * reading" and safety.c holds its previous trip state, which is the safe
     * default. */
    s_i2c_ok = true;
    ESP_LOGI(TAG, "i2c up on sda=%d scl=%d; add the VL53L1X driver at F-6 step 9",
             PIN_RANGE_SDA, PIN_RANGE_SCL);
    return true;
}

uint16_t hal_range_read_mm(void)
{
    if (!s_i2c_ok) {
        return RANGE_INVALID;
    }
    /* Vendor driver call goes here.  Returning RANGE_INVALID is honest: it
     * means "unknown", and safety.c never reads that as "clear". */
    return RANGE_INVALID;
}

/* ---- battery ------------------------------------------------------ */

bool hal_vbat_init(void)
{
    adc_oneshot_unit_init_cfg_t unit = {.unit_id = ADC_UNIT_1};
    if (adc_oneshot_new_unit(&unit, &s_adc) != ESP_OK) {
        return false;
    }
    adc_oneshot_chan_cfg_t chan = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_oneshot_config_channel(s_adc, ADC_CHANNEL_0, &chan) != ESP_OK) {
        return false;
    }
    s_adc_ok = true;
    return true;
}

uint16_t hal_vbat_read_mv(void)
{
    if (!s_adc_ok) {
        return 0;
    }
    int raw = 0;
    if (adc_oneshot_read(s_adc, ADC_CHANNEL_0, &raw) != ESP_OK) {
        return 0;
    }
    /* 12-bit at 12 dB is roughly 0..3100 mV.  Scale by the divider measured
     * during calibration, not by the nominal resistor values. */
    int adc_mv = (raw * 3100) / 4095;
    return (uint16_t)((adc_mv * VBAT_DIVIDER_NUM) / VBAT_DIVIDER_DEN);
}

void hal_led_set(bool on)
{
    if (PIN_STATUS_LED < 0) {
        return;
    }
    gpio_set_level(PIN_STATUS_LED, on ? 1 : 0);
}

#endif /* !ROVER_HOST_TEST */
