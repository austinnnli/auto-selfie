# ESP-IDF top-level project file.
#
#   idf.py set-target esp32s3
#   idf.py build flash monitor
#
# Before building, regenerate the compiled-in defaults from config.json:
#   python ../tools/gen_config_header.py
#
# Wi-Fi credentials are compiled in; override without editing a tracked file:
#   idf.py -DROVER_WIFI_SSID='"my-net"' -DROVER_WIFI_PASS='"secret"' build

cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(framing_rover)
