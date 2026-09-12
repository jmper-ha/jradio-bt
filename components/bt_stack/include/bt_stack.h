#pragma once

/* The Bluetooth controller and Bluedroid, brought up once at boot in
 * BR/EDR-only mode. The profiles (A2DP sink, A2DP source, AVRCP) register on
 * top of this in their own files; this one only owns the stack's lifetime
 * and the device name. */

#include <stdbool.h>

#include "esp_err.h"

esp_err_t bt_stack_start(const char *device_name);

/* Renames the device; takes effect for the next connection. */
esp_err_t bt_stack_set_name(const char *device_name);

/* Discoverable and connectable, or connectable only. */
esp_err_t bt_stack_set_discoverable(bool discoverable);
