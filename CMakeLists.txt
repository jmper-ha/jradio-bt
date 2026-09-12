cmake_minimum_required(VERSION 3.16)

# jradio-bt: the Bluetooth audio module beside jRadio - an ESP32 (classic,
# the one with Bluetooth Classic) that does A2DP both ways and talks to the
# S3 over one UART. See README.md.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(jradio-bt)
