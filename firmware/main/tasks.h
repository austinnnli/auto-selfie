/* tasks.h - the control tasks, shared by both entry points.
 *
 * There are two ways to build this firmware:
 *
 *   firmware/                     ESP-IDF   -> main.c        -> app_main()
 *   firmware_arduino/             Arduino   -> *.ino         -> setup()
 *
 * Everything below the entry point is identical, so it lives here rather than
 * in either one.  A task body copied into two places is a task body that will
 * differ by next month.
 */

#ifndef ROVER_TASKS_H
#define ROVER_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Configure the pins, PWM, stepper and safety checks from the compiled-in
 * defaults.  Call before net_init(). */
void rover_init_peripherals(void);

/* Create the five tasks with the core pinning and priorities from F-1.
 * Call after net_init(), which must have joined Wi-Fi first. */
void rover_start_tasks(void);

#ifdef __cplusplus
}
#endif

#endif /* ROVER_TASKS_H */
