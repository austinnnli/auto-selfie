/* Arduino.h - just enough of the Arduino-ESP32 core to COMPILE the sketch on
 * a host.  Not a simulator: nothing here runs anything.
 *
 * Its whole job is to catch, at a desk, the mistakes that otherwise surface as
 * a wall of red in the Arduino IDE twenty minutes into a build: a typo, a
 * missing include, a function that moved between core 2.x and 3.x, a C symbol
 * called from C++ without extern "C".
 */
#ifndef ARDUINO_H_STUB
#define ARDUINO_H_STUB

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/* Pick the core version being emulated; the sketch supports both. */
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 3
#endif

#define HIGH 1
#define LOW 0
#define OUTPUT 0x03
#define ADC_11db 3

#ifdef __cplusplus
extern "C" {
#endif

void pinMode(int pin, int mode);
void digitalWrite(int pin, int level);
void delay(uint32_t ms);
uint32_t millis(void);
void analogReadResolution(int bits);
void analogSetPinAttenuation(int pin, int attenuation);
uint32_t analogReadMilliVolts(int pin);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
bool ledcAttach(int pin, uint32_t freq, uint8_t resolution);
bool ledcWrite(int pin, uint32_t duty);
#else
double ledcSetup(uint8_t channel, double freq, uint8_t resolution);
void ledcAttachPin(int pin, uint8_t channel);
void ledcWrite(uint8_t channel, uint32_t duty);
#endif

#ifdef __cplusplus
}
#endif

#define log_i(fmt, ...) ((void)0)
#define log_w(fmt, ...) ((void)0)
#define log_e(fmt, ...) ((void)0)

#ifdef __cplusplus
class SerialStub {
public:
    void begin(unsigned long baud);
    void print(const char *s);
    void print(char c);
    void println();
    void println(const char *s);
    template <typename T> void println(T value) { (void)value; }
    void printf(const char *fmt, ...);
};
extern SerialStub Serial;
#endif /* __cplusplus */

/* --- FreeRTOS, as the core re-exports it ------------------------------- */
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
typedef struct { int owner; } portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {0}
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#ifdef __cplusplus
extern "C" {
#endif
void portENTER_CRITICAL(portMUX_TYPE *mux);
void portEXIT_CRITICAL(portMUX_TYPE *mux);
void vTaskDelay(TickType_t ticks);
void vTaskDelayUntil(TickType_t *prev, TickType_t period);
TickType_t xTaskGetTickCount(void);
int xTaskCreatePinnedToCore(void (*fn)(void *), const char *name,
                            uint32_t stack, void *arg, unsigned prio,
                            TaskHandle_t *handle, int core);
#ifdef __cplusplus
}
#endif

#endif
