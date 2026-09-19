/* app_config.h - pin map and compiled-in defaults.
 *
 * H-1 PIN BUDGET - READ THIS BEFORE FLASHING ANYTHING.
 *
 * "ESP32-S3-CAM" covers several board layouts.  The pins below are a starting
 * point for a board with NO camera module plugged in; you MUST check them
 * against your own board's schematic before wiring.  Reserved on every S3:
 *
 *     GPIO 0, 3, 45, 46   strapping
 *     GPIO 19, 20         USB
 *     GPIO 43, 44         UART0
 *     GPIO 26..32         SPI flash
 *     GPIO 33..37         octal PSRAM, where fitted
 *
 * The ROVER_PIN_RESERVED() static assertions at the bottom of this file catch
 * the common mistakes at compile time.  They cannot know about your board's
 * camera socket or on-board LED, so they are a backstop, not a substitute for
 * reading the schematic.
 *
 * The firmware needs 10 GPIO: 2 PWM, 4 direction, 4 stepper.  If your board
 * cannot spare 10, move the stepper's four lines to a PCF8574 I2C expander and
 * define ROVER_TILT_VIA_PCF8574 - the 28BYJ-48 tops out near 500 half-steps per
 * second, well inside what a 100 kHz I2C bus can drive.
 *
 * The camera is NOT used and its driver is never initialised (H-table, F-5).
 */

#ifndef ROVER_APP_CONFIG_H
#define ROVER_APP_CONFIG_H

#include "config_defaults.h"   /* generated from config.json by tools/gen_config_header.py */

/* ------------------------------------------------------------------ */
/* H-2  Control lines are SHARED across both L298N boards.            */
/* Do not wire twelve pins.  Each board drives one side of the rover, */
/* with its two channels wired in parallel to that side's two motors. */
/*                                                                    */
/*   PWM_L      -> board 1 ENA + ENB                                  */
/*   DIR_L_A/B  -> board 1 IN1+IN3 / IN2+IN4                          */
/*   PWM_R      -> board 2 ENA + ENB                                  */
/*   DIR_R_A/B  -> board 2 IN1+IN3 / IN2+IN4                          */
/*                                                                    */
/* (DIR_A, DIR_B) = (1,0) forward, (0,1) reverse, (0,0) coast,        */
/*                  (1,1) brake                                       */
/* ------------------------------------------------------------------ */
#define PIN_PWM_L    4
#define PIN_DIR_L_A  5
#define PIN_DIR_L_B  6
#define PIN_PWM_R    7
#define PIN_DIR_R_A  15
#define PIN_DIR_R_B  16

/* 28BYJ-48 via ULN2003.  Four unipolar phases, no step/dir input - the
 * firmware generates the sequence itself (H-table, F-3). */
#define PIN_TILT_IN1 17
#define PIN_TILT_IN2 18
#define PIN_TILT_IN3 8
#define PIN_TILT_IN4 9

/* VL53L1X over I2C.  For an HC-SR04 instead, define ROVER_RANGE_ULTRASONIC
 * and these become TRIG and ECHO. */
#define PIN_RANGE_SDA 10
#define PIN_RANGE_SCL 11
#define VL53L1X_ADDR  0x29

/* Battery sense through a divider into ADC1.  GPIO1 = ADC1_CH0. */
#define PIN_VBAT_ADC  1
#define VBAT_DIVIDER_NUM 3   /* vbat_mv = adc_mv * NUM / DEN, set by measurement */
#define VBAT_DIVIDER_DEN 1

/* Status LED.  Board-specific; set to -1 if your board has none free. */
#define PIN_STATUS_LED 48

/* ------------------------------------------------------------------ */
/* LEDC                                                               */
/* H-3: 1 kHz on ENA/ENB.  Do NOT run these bridges at 20 kHz.        */
/* ------------------------------------------------------------------ */
#define LEDC_TIMER_BITS   10          /* 0..1023 */
#define LEDC_MAX_DUTY     ((1 << LEDC_TIMER_BITS) - 1)
#define LEDC_CHANNEL_LEFT  0
#define LEDC_CHANNEL_RIGHT 1

/* ------------------------------------------------------------------ */
/* Task rates (F-1)                                                   */
/* ------------------------------------------------------------------ */
#define DRIVE_TASK_HZ      100
#define SAFETY_TASK_HZ     100
#define TELEMETRY_TASK_HZ  20
#define TILT_TICK_HZ       1000

#define DRIVE_TASK_PERIOD_MS     (1000 / DRIVE_TASK_HZ)
#define SAFETY_TASK_PERIOD_MS    (1000 / SAFETY_TASK_HZ)
#define TELEMETRY_TASK_PERIOD_MS (1000 / TELEMETRY_TASK_HZ)

/* Core pinning: Wi-Fi on core 0, control on core 1, so a network stall
 * cannot delay a motor update (F-1). */
#define CORE_NET     0
#define CORE_CONTROL 1

#define TASK_PRIO_NET       6
#define TASK_PRIO_DRIVE     8
#define TASK_PRIO_TILT      9
#define TASK_PRIO_SAFETY    10
#define TASK_PRIO_TELEMETRY 5

/* ------------------------------------------------------------------ */
/* Networking                                                         */
/* ------------------------------------------------------------------ */
#ifndef ROVER_WIFI_SSID
#define ROVER_WIFI_SSID "rover-net"
#endif
#ifndef ROVER_WIFI_PASS
#define ROVER_WIFI_PASS "changeme123"
#endif
#define ROVER_CMD_PORT 3333
#define ROVER_TLM_PORT 3334

/* ------------------------------------------------------------------ */
/* Compile-time pin sanity (H-1)                                      */
/* ------------------------------------------------------------------ */
#define ROVER_PIN_RESERVED(p)                                                 \
    ((p) == 0 || (p) == 3 || (p) == 45 || (p) == 46 ||   /* strapping */      \
     (p) == 19 || (p) == 20 ||                            /* USB       */      \
     (p) == 43 || (p) == 44 ||                            /* UART0     */      \
     ((p) >= 26 && (p) <= 32))                            /* flash     */

#ifdef ROVER_OCTAL_PSRAM
#define ROVER_PIN_RESERVED_PSRAM(p) ((p) >= 33 && (p) <= 37)
#else
#define ROVER_PIN_RESERVED_PSRAM(p) (0)
#endif

#define ROVER_PIN_OK(p) (!ROVER_PIN_RESERVED(p) && !ROVER_PIN_RESERVED_PSRAM(p))

#define ROVER_CHECK_PIN(p) \
    _Static_assert(ROVER_PIN_OK(p), #p " is a reserved GPIO on the ESP32-S3 - see H-1 in app_config.h")

ROVER_CHECK_PIN(PIN_PWM_L);
ROVER_CHECK_PIN(PIN_DIR_L_A);
ROVER_CHECK_PIN(PIN_DIR_L_B);
ROVER_CHECK_PIN(PIN_PWM_R);
ROVER_CHECK_PIN(PIN_DIR_R_A);
ROVER_CHECK_PIN(PIN_DIR_R_B);
ROVER_CHECK_PIN(PIN_TILT_IN1);
ROVER_CHECK_PIN(PIN_TILT_IN2);
ROVER_CHECK_PIN(PIN_TILT_IN3);
ROVER_CHECK_PIN(PIN_TILT_IN4);
ROVER_CHECK_PIN(PIN_RANGE_SDA);
ROVER_CHECK_PIN(PIN_RANGE_SCL);

#endif /* ROVER_APP_CONFIG_H */
