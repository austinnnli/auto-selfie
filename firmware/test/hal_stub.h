/* hal_stub.h - recorded state for the host test build. */

#ifndef ROVER_HAL_STUB_H
#define ROVER_HAL_STUB_H

#include <stdbool.h>
#include <stdint.h>

#include "rover_hal.h"

#define HAL_STUB_MAX_PIN 64
#define HAL_STUB_MAX_CHAN 4

typedef struct {
    int64_t now_us;
    int pin_level[HAL_STUB_MAX_PIN];     /* -1 never written, else 0/1 */
    int pin_configured[HAL_STUB_MAX_PIN];
    uint32_t pwm_duty[HAL_STUB_MAX_CHAN];
    int pwm_freq;
    int pwm_bits;
    int pwm_inited;
    unsigned pin_writes;
    unsigned pwm_writes;
    uint16_t range_mm;
    uint16_t vbat_mv;
    bool range_present;
    bool vbat_present;
    int led;
} hal_stub_t;

extern hal_stub_t g_hal;

void hal_stub_reset(void);
void hal_stub_advance_us(int64_t us);

#endif /* ROVER_HAL_STUB_H */
