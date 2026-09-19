idf_component_register(
    SRCS
        "main.c"
        "tasks.c"
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

# Wi-Fi credentials without committing them.
#
#     idf.py -DROVER_WIFI_SSID=my-net -DROVER_WIFI_PASS=secret build
#
# Pass the values unquoted; the quoting for C is added here.  Without this
# plumbing the -D would only set a CMake cache variable and never reach the
# compiler, so app_config.h's defaults would be used and the rover would
# quietly try to join "rover-net" instead.
foreach(rover_var ROVER_WIFI_SSID ROVER_WIFI_PASS)
    if(DEFINED ${rover_var})
        target_compile_definitions(${COMPONENT_LIB} PRIVATE
            "${rover_var}=\"${${rover_var}}\"")
    endif()
endforeach()
