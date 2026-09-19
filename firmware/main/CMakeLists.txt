idf_component_register(
    SRCS
        "main.c"
        "net.c"
        "drive.c"
        "tilt.c"
        "safety.c"
        "hal_esp32.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_adc
        esp_timer
        esp_wifi
        esp_netif
        esp_event
        nvs_flash
        lwip
)

# The camera component is deliberately absent.  The ESP32-S3-CAM's camera is
# NOT used and its driver is never initialised (hardware table, F-5).
