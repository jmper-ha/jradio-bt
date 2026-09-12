#pragma once

/* The phone-to-module half: A2DP sink for the audio, AVRCP controller for
 * the track and the transport keys, AVRCP target for the phone's volume
 * slider. PCM goes straight to audio_out; everything else is reported to
 * the listener, which is main's job to turn into frames for the host.
 *
 * The listener runs on Bluedroid's own task: keep it to state and UART
 * writes, never a call back into the stack. */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "jbt_proto.h"

#define A2DP_TEXT_MAX 255U

typedef struct {
    char title[A2DP_TEXT_MAX + 1U];
    char artist[A2DP_TEXT_MAX + 1U];
    char album[A2DP_TEXT_MAX + 1U];
    char genre[A2DP_TEXT_MAX + 1U];
    uint32_t duration_ms;
    uint32_t track_no;
} a2dp_track_t;

typedef struct {
    void (*connection)(jbt_conn_t state, const uint8_t *address);
    void (*peer_name)(const char *name);
    void (*audio_format)(jbt_codec_t codec, uint32_t sample_rate);
    void (*play)(jbt_play_t state);
    void (*track)(const a2dp_track_t *track);
    void (*position)(uint32_t position_ms);
    /* The phone moved its slider. */
    void (*volume)(uint8_t volume);
    /* A cover has arrived (size > 0) or the track has none (size 0). The
     * bytes stay with the sink until the next cover; a2dp_sink_cover_read()
     * hands them out in pieces. */
    void (*cover)(uint32_t size, jbt_image_t kind, uint32_t hash);
} a2dp_sink_listener_t;

/* Copies up to `max` bytes of the current cover from `offset` into `out`;
 * returns how many, 0 past the end or with no cover. `hash` names the cover
 * the bytes belong to, so a reader can tell it changed under them. */
size_t a2dp_sink_cover_read(uint32_t offset, uint8_t *out, size_t max, uint32_t *hash);

/* Registers the profiles with a running stack. Once per boot. */
esp_err_t a2dp_sink_start(const a2dp_sink_listener_t *listener);

/* Connectable or not: in sink mode a phone may come back on its own; off,
 * nothing may connect and a live connection is dropped. */
void a2dp_sink_set_enabled(bool enabled);

esp_err_t a2dp_sink_connect(const uint8_t *address);
esp_err_t a2dp_sink_disconnect(void);

/* A transport key to the phone, press and release. */
esp_err_t a2dp_sink_passthrough(jbt_key_t key);

/* The host turned the knob: applied here and told to the phone, whose
 * slider follows. */
void a2dp_sink_set_volume(uint8_t volume);
