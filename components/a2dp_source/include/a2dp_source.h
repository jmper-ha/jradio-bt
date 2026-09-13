#pragma once

/* The module-to-speaker half: A2DP source, fed from audio_in.
 *
 * Bluedroid runs one A2DP role at a time, so this and a2dp_sink are never
 * up together: entering source mode takes the sink down first (a2dp_sink
 * has no "stop" of its own - main asks the sink to deinit and this to
 * init on the DEINIT event), and leaving it runs the other way.
 *
 * The listener runs on Bluedroid's task: keep it to state and UART. */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "jbt_proto.h"

typedef struct {
    uint8_t address[6];
    int8_t rssi;
    uint32_t class_of_device;
    char name[64];
} a2dp_scan_result_t;

typedef struct {
    void (*connection)(jbt_conn_t state, const uint8_t *address);
    void (*peer_name)(const char *name);
    /* One per device found; `scanning` false when the scan ends. */
    void (*scan_result)(const a2dp_scan_result_t *result);
    void (*scanning)(bool scanning);
    /* The stream started or stopped, as the speaker sees it. */
    void (*play)(jbt_play_t state);
    void (*profile)(bool up);
    /* A button on the speaker: play/pause, next, previous. */
    void (*key)(jbt_key_t key);
    /* The speaker set its own volume and told us (absolute volume). */
    void (*volume)(uint8_t volume);
} a2dp_source_listener_t;

/* Registers the profile. The stack must not have the sink up. */
esp_err_t a2dp_source_start(const a2dp_source_listener_t *listener);
/* Takes the profile down; the DEINIT event comes back through Bluedroid
 * and main may then bring the sink up again. */
esp_err_t a2dp_source_stop(void);

/* Looks for speakers and headphones (rendering devices) for `seconds`. */
esp_err_t a2dp_source_scan(bool on);
esp_err_t a2dp_source_connect(const uint8_t *address);
esp_err_t a2dp_source_disconnect(void);

/* The speaker's own volume, over AVRCP absolute volume when it offers it. */
esp_err_t a2dp_source_set_volume(uint8_t volume);
