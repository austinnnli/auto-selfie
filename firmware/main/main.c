/* main.c - the ESP-IDF entry point.
 *
 * Everything of substance is in tasks.c, which the Arduino build in
 * firmware_arduino/ shares.  This file exists only to be app_main().
 */

#include "net.h"
#include "tasks.h"

void app_main(void)
{
    rover_init_peripherals();
    net_init();
    rover_start_tasks();
}
