/* main.c - task setup and core pinning.
 *
 * F-1.  Wi-Fi is pinned to core 0 and control to core 1, so a network stall
 * cannot delay a motor update.
 *
 *   task       core  rate        job
 *   net         0    event       receive, validate CRC and sequence
 *   drive       1    100 Hz      apply duty with ramping
 *   tilt        1    1 kHz tick  advance one half-step when due
 *   safety      1    100 Hz      watchdog, range check, enable gate
 *   telemetry   0    20 Hz       send state back to the laptop
 *
 * The firmware is deliberately stupid.  It applies what it is told, reports
 * what it sees, and stops when it stops hearing.  No control logic lives here
 * and the camera driver is never initialised (F-5).
 */

#include "app_config.h"
#include "drive.h"
#include "net.h"
#include "protocol.h"
#include "rover_hal.h"
#include "safety.h"
#include "tilt.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "rover";

/* ------------------------------------------------------------------ */
/* drive task - 100 Hz                                                */
/* ------------------------------------------------------------------ */
static void drive_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(DRIVE_TASK_PERIOD_MS);
    TickType_t last = xTaskGetTickCount();
    int64_t prev_us = esp_timer_get_time();

    while (1) {
        vTaskDelayUntil(&last, period);

        /* W-5.5: loop_us is the early-warning signal.  If this grows, the
         * firmware is being starved and the watchdog margin is shrinking. */
        int64_t now_us = esp_timer_get_time();
        net_report_loop_us((uint32_t)(now_us - prev_us));
        prev_us = now_us;

        net_command_t cmd;
        bool have = net_get_command(&cmd);

        safety_status_t st;
        safety_get(&st);

        if (!have || !safety_motion_allowed(cmd.enable, &st)) {
            drive_coast();
            drive_tick();
            continue;
        }

        int left = cmd.left;
        int right = cmd.right;

        /* F-4: a range trip refuses forward motion but leaves reverse and
         * turns available, so the rover can always back out of a corner. */
        if (st.range_trip) {
            safety_limit_forward(left, right, &left, &right);
        }

        drive_set(left, right);
        drive_tick();
    }
}

/* ------------------------------------------------------------------ */
/* tilt task - 1 kHz tick                                             */
/* ------------------------------------------------------------------ */
static void tilt_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(1000 / TILT_TICK_HZ);
    TickType_t last = xTaskGetTickCount();
    uint16_t last_seq = 0;
    bool seen = false;

    while (1) {
        vTaskDelayUntil(&last, period);

        net_command_t cmd;
        if (net_get_command(&cmd)) {
            if (!seen || cmd.seq != last_seq) {
                last_seq = cmd.seq;
                seen = true;

                if (cmd.zero_tilt) {
                    /* The laptop has measured true pitch from the phone's
                     * gravity vector and is telling us what we are actually
                     * looking at.  No homing routine exists or is wanted. */
                    tilt_zero_at(cmd.tilt_deg);
                } else if (cmd.tilt_release) {
                    tilt_release();
                } else {
                    tilt_set_target_deg(cmd.tilt_deg);
                }
            }
        }
        tilt_tick();
    }
}

/* ------------------------------------------------------------------ */
/* safety task - 100 Hz                                               */
/* ------------------------------------------------------------------ */
static void safety_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(SAFETY_TASK_PERIOD_MS);
    TickType_t last = xTaskGetTickCount();
    bool led = false;

    while (1) {
        vTaskDelayUntil(&last, period);
        safety_tick();

        /* Slow blink when stopped, solid when armed - visible across a field. */
        safety_status_t st;
        safety_get(&st);
        static int count;
        if (st.watchdog_trip || st.vbat_low) {
            if (++count >= SAFETY_TASK_HZ / 4) {
                count = 0;
                led = !led;
                hal_led_set(led);
            }
        } else if (!led) {
            led = true;
            hal_led_set(true);
        }
    }
}

/* ------------------------------------------------------------------ */
/* boot                                                               */
/* ------------------------------------------------------------------ */
void app_main(void)
{
    ESP_LOGI(TAG, "framing rover firmware, protocol v%d, config %s",
             PROTOCOL_VERSION, CFG_HASH);
    ESP_LOGI(TAG, "camera driver is NOT initialised - the phone is the camera");

    hal_gpio_config_output(PIN_STATUS_LED);

    const drive_cfg_t dcfg = {
        .min_duty_left = CFG_MIN_DUTY_LEFT,
        .min_duty_right = CFG_MIN_DUTY_RIGHT,
        .max_duty = CFG_MAX_DUTY,
        .max_duty_per_tick = CFG_MAX_DUTY_PER_TICK,
        .trim = CFG_TRIM,
        .pwm_freq = CFG_PWM_FREQ,
    };
    drive_init(&dcfg);

    const tilt_cfg_t tcfg = {
        .steps_per_degree = CFG_STEPS_PER_DEGREE,
        .backlash_steps = CFG_BACKLASH_STEPS,
        .max_step_rate = CFG_MAX_STEP_RATE,
        .release_ms = CFG_TILT_RELEASE_MS,
        .min_deg = CFG_TILT_MIN_DEG,
        .max_deg = CFG_TILT_MAX_DEG,
    };
    tilt_init(&tcfg);

    const safety_cfg_t scfg = {
        .watchdog_ms = CFG_WATCHDOG_MS,
        .range_stop_mm = CFG_RANGE_STOP_MM,
        .vbat_min_mv = CFG_VBAT_MIN_MV,
    };
    safety_init(&scfg);

    net_init();

    /* Control on core 1.  Safety runs at the highest priority of the three so
     * it cannot be starved by the tilt tick. */
    xTaskCreatePinnedToCore(safety_task, "safety", 3072, NULL,
                            TASK_PRIO_SAFETY, NULL, CORE_CONTROL);
    xTaskCreatePinnedToCore(tilt_task, "tilt", 3072, NULL,
                            TASK_PRIO_TILT, NULL, CORE_CONTROL);
    xTaskCreatePinnedToCore(drive_task, "drive", 3072, NULL,
                            TASK_PRIO_DRIVE, NULL, CORE_CONTROL);

    /* Network on core 0, with the Wi-Fi stack. */
    xTaskCreatePinnedToCore(net_rx_task, "net_rx", 4096, NULL,
                            TASK_PRIO_NET, NULL, CORE_NET);
    xTaskCreatePinnedToCore(net_telemetry_task, "net_tlm", 4096, NULL,
                            TASK_PRIO_TELEMETRY, NULL, CORE_NET);

    ESP_LOGI(TAG, "tasks started; motors stay stopped until a valid packet arrives");
}
