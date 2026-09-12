#include "bt_stack.h"

#include <stdbool.h>

#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_check.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"

static const char *TAG = "bt_stack";

esp_err_t bt_stack_start(const char *device_name)
{
    /* BLE's share of the controller memory is given back: this module never
     * advertises, and A2DP wants the RAM. */
    ESP_RETURN_ON_ERROR(esp_bt_controller_mem_release(ESP_BT_MODE_BLE), TAG, "release ble");
    esp_bt_controller_config_t config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_bt_controller_init(&config), TAG, "controller init");
    ESP_RETURN_ON_ERROR(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT), TAG, "controller enable");
    ESP_RETURN_ON_ERROR(esp_bluedroid_init(), TAG, "bluedroid init");
    ESP_RETURN_ON_ERROR(esp_bluedroid_enable(), TAG, "bluedroid enable");
    ESP_RETURN_ON_ERROR(bt_stack_set_name(device_name), TAG, "set name");
    /* Connectable so a phone that already knows us can come back; not
     * discoverable until the host asks, the way a speaker behaves. */
    ESP_RETURN_ON_ERROR(bt_stack_set_discoverable(false), TAG, "scan mode");
    const uint8_t *address = esp_bt_dev_get_address();
    ESP_LOGI(TAG, "up as \"%s\", %02X:%02X:%02X:%02X:%02X:%02X", device_name, address[0],
             address[1], address[2], address[3], address[4], address[5]);
    return ESP_OK;
}

esp_err_t bt_stack_set_name(const char *device_name)
{
    return esp_bt_gap_set_device_name(device_name);
}

esp_err_t bt_stack_set_discoverable(bool discoverable)
{
    return esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, discoverable ? ESP_BT_GENERAL_DISCOVERABLE
                                                                      : ESP_BT_NON_DISCOVERABLE);
}
