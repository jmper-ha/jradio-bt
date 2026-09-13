/* jradio-bt: the Bluetooth audio module beside jRadio.
 *
 * This file is the dispatcher: a frame from the host comes in on the link
 * task, is answered from the module's state, and the Bluetooth profiles are
 * told what to do; what they report comes back through the listener at the
 * bottom and goes out as frames. Everything the host can ask is listed in
 * jbt_proto.h; what is not implemented yet answers UNSUPPORTED rather than
 * silence, so the host's timeouts never fire on a known command. */

#include <string.h>

#include "a2dp_sink.h"
#include "audio_out.h"
#include "bt_stack.h"
#include "esp_log.h"
#include "jbt_link.h"
#include "jbt_proto.h"
#include "module_state.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "version.h"

static const char *TAG = "jbt";

static void send_status(void)
{
    module_state_t state;
    module_state_get(&state);
    uint8_t payload[JBT_STATUS_FIXED_SIZE + 2U + MODULE_PEER_NAME_MAX];
    jbt_writer_t writer;
    jbt_writer_init(&writer, payload, sizeof(payload));
    jbt_put_status(&writer, &state.status);
    if (state.peer_name[0] != '\0') jbt_put_tlv_string(&writer, JBT_TAG_PEER_NAME, state.peer_name);
    if (!writer.overflow) (void)jbt_link_send(JBT_MSG_STATUS, 0U, payload, writer.length, NULL);
}

static void send_event(jbt_event_t event, const char *text)
{
    uint8_t payload[3U + 64U];
    jbt_writer_t writer;
    jbt_writer_init(&writer, payload, sizeof(payload));
    jbt_put_u8(&writer, (uint8_t)event);
    if (text != NULL) jbt_put_tlv_string(&writer, JBT_TAG_TEXT, text);
    if (!writer.overflow) (void)jbt_link_send(JBT_MSG_EVENT, 0U, payload, writer.length, NULL);
}

static void send_pong(void)
{
    uint8_t payload[5];
    jbt_writer_t writer;
    jbt_writer_init(&writer, payload, sizeof(payload));
    jbt_put_u8(&writer, JBT_PROTOCOL_VERSION);
    jbt_put_u8(&writer, JBT_FW_MAJOR);
    jbt_put_u8(&writer, JBT_FW_MINOR);
    jbt_put_u16(&writer, JBT_FW_BUILD);
    (void)jbt_link_send(JBT_MSG_PONG, 0U, payload, writer.length, NULL);
}

static void send_mode_ack(jbt_mode_t mode, jbt_result_t result)
{
    const uint8_t payload[2] = {(uint8_t)mode, (uint8_t)result};
    (void)jbt_link_send(JBT_MSG_MODE_ACK, 0U, payload, sizeof(payload), NULL);
}

/* SET_MODE is the one command with a handshake of its own: MODE_ACK says the
 * I2S bus has changed hands, and the host waits for it before touching its
 * own pins. Going to sink, the host has already let go of the bus when it
 * asks; going to off, it takes the bus back only after the ack. So the bus
 * is claimed before the ack on the way in and released before it on the way
 * out, and there is never a moment with two drivers. */
static jbt_result_t enter_mode(jbt_mode_t mode)
{
    module_state_t state;
    module_state_get(&state);
    if (state.status.mode == mode) return JBT_RESULT_OK;
    /* Leave the old mode first. */
    if (state.status.mode == JBT_MODE_SINK) {
        a2dp_sink_set_enabled(false);
        audio_out_release();
    }
    jbt_result_t result = JBT_RESULT_OK;
    if (mode == JBT_MODE_SINK) {
        const esp_err_t err = audio_out_claim();
        if (err != ESP_OK) {
            /* Reported as off, which is what the host must assume about the
             * bus when the claim failed. */
            ESP_LOGE(TAG, "cannot claim the bus: %s", esp_err_to_name(err));
            mode = JBT_MODE_OFF;
            result = JBT_RESULT_FAILED;
        } else {
            a2dp_sink_set_enabled(true);
            /* Back to the phone that was here last: an iPhone waits to be
             * called, it does not call - so after a reboot of the module
             * mid-play the music stayed off until somebody tapped it. */
            uint8_t peer[6];
            if (module_state_last_peer(peer)) {
                ESP_LOGI(TAG, "calling the last phone %02X:%02X:%02X:%02X:%02X:%02X", peer[0],
                         peer[1], peer[2], peer[3], peer[4], peer[5]);
                (void)a2dp_sink_connect(peer);
            }
        }
    }
    module_state_set_mode(mode);
    module_state_set_connection(JBT_CONN_NONE, NULL, NULL);
    module_state_set_play(JBT_PLAY_STOPPED);
    ESP_LOGI(TAG, "mode %d", mode);
    return result;
}

static void handle_set_mode(const jbt_frame_t *frame)
{
    jbt_reader_t reader;
    jbt_reader_init(&reader, frame->payload, frame->len);
    uint8_t mode = 0;
    if (!jbt_get_u8(&reader, &mode)) {
        send_mode_ack(JBT_MODE_OFF, JBT_RESULT_BAD_ARG);
        return;
    }
    switch ((jbt_mode_t)mode) {
    case JBT_MODE_OFF:
    case JBT_MODE_SINK: {
        const jbt_result_t result = enter_mode((jbt_mode_t)mode);
        module_state_t state;
        module_state_get(&state);
        send_mode_ack((jbt_mode_t)state.status.mode, result);
        send_status();
        return;
    }
    case JBT_MODE_SOURCE:
        send_mode_ack((jbt_mode_t)mode, JBT_RESULT_UNSUPPORTED);
        return;
    }
    send_mode_ack(JBT_MODE_OFF, JBT_RESULT_BAD_ARG);
}

static void handle_connect(const jbt_frame_t *frame)
{
    jbt_result_t result = JBT_RESULT_BAD_ARG;
    if (frame->len == 6U) {
        result = a2dp_sink_connect(frame->payload) == ESP_OK ? JBT_RESULT_OK : JBT_RESULT_FAILED;
    }
    if (frame->flags & JBT_FLAG_WANT_ACK) (void)jbt_link_ack(frame->seq, result);
}

static void handle_passthrough(const jbt_frame_t *frame)
{
    jbt_result_t result = JBT_RESULT_BAD_ARG;
    if (frame->len == 1U) {
        result = a2dp_sink_passthrough((jbt_key_t)frame->payload[0]) == ESP_OK ? JBT_RESULT_OK
                                                                                : JBT_RESULT_FAILED;
    }
    if (frame->flags & JBT_FLAG_WANT_ACK) (void)jbt_link_ack(frame->seq, result);
}

static void handle_set_volume(const jbt_frame_t *frame)
{
    jbt_result_t result = JBT_RESULT_BAD_ARG;
    if (frame->len == 1U) {
        a2dp_sink_set_volume(frame->payload[0]);
        module_state_set_volume(frame->payload[0]);
        result = JBT_RESULT_OK;
    }
    if (frame->flags & JBT_FLAG_WANT_ACK) (void)jbt_link_ack(frame->seq, result);
}

static void handle_set_name(const jbt_frame_t *frame)
{
    jbt_reader_t reader;
    jbt_reader_init(&reader, frame->payload, frame->len);
    uint8_t tag;
    const uint8_t *value;
    uint8_t length;
    char name[MODULE_NAME_MAX + 1U] = {0};
    while (jbt_get_tlv(&reader, &tag, &value, &length)) {
        if (tag == JBT_TAG_NAME) jbt_tlv_to_string(value, length, name, sizeof(name));
    }
    jbt_result_t result = JBT_RESULT_BAD_ARG;
    if (module_state_set_name(name)) {
        result = bt_stack_set_name(name) == ESP_OK ? JBT_RESULT_OK : JBT_RESULT_FAILED;
        ESP_LOGI(TAG, "name is now \"%s\"", name);
    }
    if (frame->flags & JBT_FLAG_WANT_ACK) (void)jbt_link_ack(frame->seq, result);
}

static void handle_pairing(const jbt_frame_t *frame)
{
    jbt_reader_t reader;
    jbt_reader_init(&reader, frame->payload, frame->len);
    uint8_t on = 0;
    jbt_result_t result = JBT_RESULT_BAD_ARG;
    module_state_t state;
    module_state_get(&state);
    if (state.status.mode != JBT_MODE_SINK) {
        /* Discoverable while off would let a phone connect to a module that
         * is not driving anything. */
        result = JBT_RESULT_BUSY;
    } else if (jbt_get_u8(&reader, &on)) {
        result = bt_stack_set_discoverable(on != 0U) == ESP_OK ? JBT_RESULT_OK : JBT_RESULT_FAILED;
        if (state.status.connection != JBT_CONN_CONNECTED) {
            module_state_set_connection(on != 0U ? JBT_CONN_PAIRING : JBT_CONN_NONE, NULL, NULL);
        }
        ESP_LOGI(TAG, "pairing %s", on != 0U ? "on" : "off");
        send_status();
    }
    if (frame->flags & JBT_FLAG_WANT_ACK) (void)jbt_link_ack(frame->seq, result);
}

static void handle_cover_get(const jbt_frame_t *frame);

static void on_frame(const jbt_frame_t *frame, void *context)
{
    (void)context;
    switch ((jbt_msg_t)frame->type) {
    case JBT_MSG_PING: send_pong(); return;
    case JBT_MSG_GET_STATUS: send_status(); return;
    case JBT_MSG_SET_MODE: handle_set_mode(frame); return;
    case JBT_MSG_SET_NAME: handle_set_name(frame); return;
    case JBT_MSG_PAIRING: handle_pairing(frame); return;
    case JBT_MSG_CONNECT: handle_connect(frame); return;
    case JBT_MSG_DISCONNECT:
        (void)a2dp_sink_disconnect();
        if (frame->flags & JBT_FLAG_WANT_ACK) (void)jbt_link_ack(frame->seq, JBT_RESULT_OK);
        return;
    case JBT_MSG_PASSTHROUGH: handle_passthrough(frame); return;
    case JBT_MSG_SET_VOLUME: handle_set_volume(frame); return;
    case JBT_MSG_COVER_GET: handle_cover_get(frame); return;
    case JBT_MSG_FORGET:
        module_state_forget_peer();
        (void)a2dp_sink_disconnect();
        if (frame->flags & JBT_FLAG_WANT_ACK) (void)jbt_link_ack(frame->seq, JBT_RESULT_OK);
        return;
    case JBT_MSG_ACK: return; /* nothing the module sends asks for one yet */
    default: break;
    }
    /* Known but not built yet, or not a host command at all: say so once
     * per frame rather than leaving the host to time out. */
    ESP_LOGW(TAG, "%s (0x%02X) not handled", jbt_msg_name(frame->type), frame->type);
    if (frame->flags & JBT_FLAG_WANT_ACK) (void)jbt_link_ack(frame->seq, JBT_RESULT_UNSUPPORTED);
}

/* The sink's reports, turned into state and frames. All on Bluedroid's
 * task: nothing here waits. */
static void on_connection(jbt_conn_t state, const uint8_t *address)
{
    module_state_t current;
    module_state_get(&current);
    /* A drop while pairing was on leaves pairing on; anything else
     * replaces the state wholesale. */
    module_state_set_connection(state, state == JBT_CONN_NONE ? NULL : address,
                                state == JBT_CONN_CONNECTED ? current.peer_name : NULL);
    if (state == JBT_CONN_NONE) module_state_set_play(JBT_PLAY_STOPPED);
    if (state == JBT_CONN_CONNECTED) module_state_remember_peer(address);
    send_status();
    if (state == JBT_CONN_CONNECTED) send_event(JBT_EVENT_CONNECTED, NULL);
    if (state == JBT_CONN_NONE && current.status.connection == JBT_CONN_CONNECTED) {
        send_event(JBT_EVENT_DISCONNECTED, NULL);
    }
}

static void on_peer_name(const char *name)
{
    module_state_t current;
    module_state_get(&current);
    module_state_set_connection((jbt_conn_t)current.status.connection, current.status.peer, name);
    send_status();
}

static void on_audio_format(jbt_codec_t codec, uint32_t sample_rate)
{
    module_state_set_codec(codec, sample_rate);
    send_status();
}

static void on_play(jbt_play_t state)
{
    module_state_set_play(state);
    const uint8_t payload[1] = {(uint8_t)state};
    (void)jbt_link_send(JBT_MSG_PLAY_STATE, 0U, payload, sizeof(payload), NULL);
}

static void on_track(const a2dp_track_t *track)
{
    /* Four strings of up to 255 plus three numbers fit the payload with
     * room over; the writer refuses anything that would not. */
    uint8_t payload[JBT_PAYLOAD_MAX];
    jbt_writer_t writer;
    jbt_writer_init(&writer, payload, sizeof(payload));
    if (track->title[0] != '\0') jbt_put_tlv_string(&writer, JBT_TAG_TITLE, track->title);
    if (track->artist[0] != '\0') jbt_put_tlv_string(&writer, JBT_TAG_ARTIST, track->artist);
    if (track->album[0] != '\0') jbt_put_tlv_string(&writer, JBT_TAG_ALBUM, track->album);
    if (track->genre[0] != '\0') jbt_put_tlv_string(&writer, JBT_TAG_GENRE, track->genre);
    if (track->duration_ms != 0U) jbt_put_tlv_u32(&writer, JBT_TAG_DURATION_MS, track->duration_ms);
    if (track->track_no != 0U) jbt_put_tlv_u32(&writer, JBT_TAG_TRACK_NO, track->track_no);
    if (!writer.overflow) (void)jbt_link_send(JBT_MSG_TRACK, 0U, payload, writer.length, NULL);
    ESP_LOGI(TAG, "track \"%s\" - \"%s\"", track->artist, track->title);
}

static void on_position(uint32_t position_ms)
{
    uint8_t payload[4];
    jbt_writer_t writer;
    jbt_writer_init(&writer, payload, sizeof(payload));
    jbt_put_u32(&writer, position_ms);
    (void)jbt_link_send(JBT_MSG_POSITION, 0U, payload, writer.length, NULL);
}

static void on_cover(uint32_t size, jbt_image_t kind, uint32_t hash)
{
    /* Width and height are left at zero: the host decodes the picture and
     * learns them itself, and parsing a JPEG header here would be a second
     * decoder for two numbers nobody uses before the first. */
    uint8_t payload[13];
    jbt_writer_t writer;
    jbt_writer_init(&writer, payload, sizeof(payload));
    jbt_put_u32(&writer, size);
    jbt_put_u8(&writer, (uint8_t)kind);
    jbt_put_u32(&writer, hash);
    jbt_put_u16(&writer, 0U);
    jbt_put_u16(&writer, 0U);
    (void)jbt_link_send(JBT_MSG_COVER_INFO, 0U, payload, writer.length, NULL);
}

/* One piece of the cover, as asked: offset and up to `max` bytes, capped at
 * what a frame holds. Past the end, or with the cover gone, an empty piece
 * says so and the host stops asking. */
static void handle_cover_get(const jbt_frame_t *frame)
{
    jbt_reader_t reader;
    jbt_reader_init(&reader, frame->payload, frame->len);
    uint32_t offset = 0U;
    uint16_t max = 0U;
    if (!jbt_get_u32(&reader, &offset) || !jbt_get_u16(&reader, &max)) return;
    if (max > JBT_PAYLOAD_MAX - 4U) max = JBT_PAYLOAD_MAX - 4U;
    uint8_t payload[JBT_PAYLOAD_MAX];
    jbt_writer_t writer;
    jbt_writer_init(&writer, payload, sizeof(payload));
    jbt_put_u32(&writer, offset);
    const size_t n = a2dp_sink_cover_read(offset, &payload[4], max, NULL);
    writer.length += n;
    (void)jbt_link_send(JBT_MSG_COVER_DATA, 0U, payload, writer.length, NULL);
}

static void on_volume(uint8_t volume)
{
    module_state_set_volume(volume);
    const uint8_t payload[1] = {volume};
    (void)jbt_link_send(JBT_MSG_VOLUME, 0U, payload, sizeof(payload), NULL);
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(module_state_init(CONFIG_JBT_DEVICE_NAME));

    /* The link first: from here on every log line reaches the host too, and
     * the stack's own start-up is the first thing worth seeing there. */
    ESP_ERROR_CHECK(jbt_link_start(on_frame, NULL));
    ESP_LOGI(TAG, "jradio-bt %u.%u build %u, protocol %u", JBT_FW_MAJOR, JBT_FW_MINOR, JBT_FW_BUILD,
             JBT_PROTOCOL_VERSION);

    module_state_t state;
    module_state_get(&state);
    ESP_ERROR_CHECK(bt_stack_start(state.name));
    const a2dp_sink_listener_t listener = {
        .connection = on_connection, .peer_name = on_peer_name, .audio_format = on_audio_format,
        .play = on_play, .track = on_track, .position = on_position, .volume = on_volume,
        .cover = on_cover,
    };
    ESP_ERROR_CHECK(a2dp_sink_start(&listener));
    /* Off until the host says otherwise: the bus is the host's by default,
     * and a phone must not be able to connect to a module nobody asked for. */
    audio_out_release();

    send_event(JBT_EVENT_BOOTED, state.name);
    send_status();
}
