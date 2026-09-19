/*
 * framing_rover.ino - Arduino IDE entry point for the D1 firmware.
 *
 * ============================ READ FIRST ============================
 *
 * 1. CHECK THE PIN MAP in app_config.h against YOUR board's schematic
 *    before connecting anything.  "ESP32-S3-CAM" covers several layouts.
 *
 * 2. PUT YOUR WI-FI DETAILS in secrets.h (copy secrets_example.h).
 *
 * 3. BOARD SETTINGS - Tools menu:
 *       Board                 ESP32S3 Dev Module
 *       USB CDC On Boot       Enabled      <- or Serial prints nothing
 *       Flash Size            as fitted (usually 8MB or 16MB)
 *       PSRAM                 as fitted (OPI PSRAM on most S3-CAM boards)
 *       Partition Scheme      Default 4MB with spiffs, or Huge APP
 *       Upload Speed          921600
 *
 *    If your board has OPI PSRAM, also uncomment ROVER_OCTAL_PSRAM in
 *    app_config.h so the compile-time pin check knows GPIO 33-37 are taken.
 *
 * 4. OPEN THE SERIAL MONITOR AT 115200 after flashing.  The rover prints
 *    its IP address on joining.  The laptop needs it:
 *       config.json -> net.firmware_host, or  python -m app.run --host <ip>
 *
 * 5. NEVER power the ESP32 from the L298N's onboard 5 V regulator.  Use a
 *    separate buck converter, common ground, bulk capacitance at the ESP32.
 *
 * 6. WHEELS OFF THE GROUND for the first test of anything.
 *
 * ====================================================================
 *
 * The camera on the ESP32-S3-CAM is NOT used and its driver is never
 * initialised (F-5).  The phone is the camera.
 *
 * Most of this firmware is shared verbatim with the ESP-IDF build in
 * firmware/.  The .c/.h files beside this sketch are COPIES, refreshed by
 * `python tools/gen_arduino_sketch.py` - edit the originals under
 * firmware/main/ and regenerate, or your change will be overwritten.
 */

#include <Arduino.h>

#include "secrets.h"      // ROVER_WIFI_SSID / ROVER_WIFI_PASS, before app_config.h

extern "C" {
#include "app_config.h"
#include "net.h"
#include "tasks.h"
}

void setup()
{
    Serial.begin(115200);

    // A moment for USB CDC to enumerate, so the banner is not lost.  Not a
    // blocking wait on Serial: a rover running from a battery has no host and
    // would hang here forever.
    delay(400);

    Serial.println();
    Serial.println("framing rover - Arduino build");

    // Exactly what app_main() does in the ESP-IDF build.  Both entry points
    // share tasks.c, so there is one copy of the task logic, not two.
    rover_init_peripherals();
    net_init();
    rover_start_tasks();
}

void loop()
{
    // Nothing happens here.  All five tasks are pinned to their own cores by
    // rover_start_tasks() (F-1: Wi-Fi on core 0, control on core 1, so a
    // network stall cannot delay a motor update).  Arduino's loopTask would
    // otherwise sit spinning on core 1 competing with the control tasks, so
    // it is put to sleep rather than left to busy-wait.
    vTaskDelay(pdMS_TO_TICKS(1000));

    static uint32_t last = 0;
    if (millis() - last >= 5000) {
        last = millis();
        net_stats_t s;
        net_get_stats(&s);
        // The numbers that matter during bring-up: accepted should climb at
        // the laptop's command rate, and bad_crc/bad_version should stay at
        // zero.  A rising bad_version means protocol.h and protocol.py have
        // drifted apart.
        Serial.printf("rx %lu ok=%lu crc=%lu ver=%lu stale=%lu | tx %lu\n",
                      (unsigned long)s.rx_total, (unsigned long)s.rx_accepted,
                      (unsigned long)s.rx_bad_crc, (unsigned long)s.rx_bad_version,
                      (unsigned long)s.rx_stale_seq, (unsigned long)s.tx_telemetry);
    }
}
