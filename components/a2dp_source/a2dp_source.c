#include "a2dp_source.h"

#include <stdio.h>
#include <string.h>

#include "audio_in.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_bt_defs.h"
#include "esp_check.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "a2dp_source";

/* How long one scan runs: the inquiry length is in units of 1.28 s. */
#define A2DP_SCAN_LENGTH 8
/* The stream is started when the bus is clocked and stopped when it has
 * gone quiet, checked on this period. Bluedroid's media control is a
 * request/ack pair, so the state below is the request in flight. */
#define A2DP_MEDIA_TICK_US (500 * 1000)

typedef enum {
    MEDIA_IDLE,
    MEDIA_CHECKING,
    MEDIA_STARTING,
    MEDIA_STARTED,
    MEDIA_STOPPING,
} media_state_t;

static a2dp_source_listener_t s_listener;
static bool s_up;
static bool s_connected;
static esp_bd_addr_t s_peer;
static bool s_scanning;
static media_state_t s_media;
static esp_timer_handle_t s_media_timer;
static uint8_t s_transaction;
static bool s_peer_takes_volume;

static uint8_t a2dp_next_transaction(void)
{
    s_transaction = (uint8_t)((s_transaction + 1U) & 0x0FU);
    return s_transaction;
}

/* The encoder's pull: 44.1 kHz stereo, as much as the bus delivered,
 * silence for the rest. A negative length is the stack asking for a
 * flush, which the ring does not need. */
static int32_t a2dp_data_callback(uint8_t *data, int32_t length)
{
    if (data == NULL || length <= 0) return 0;
    (void)audio_in_read(data, (size_t)length);
    return length;
}

static void a2dp_media_tick(void *arg)
{
    (void)arg;
    if (!s_connected) return;
    const bool clocked = audio_in_clocked();
    switch (s_media) {
    case MEDIA_IDLE:
        if (clocked) {
            s_media = MEDIA_CHECKING;
            (void)esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY);
        }
        break;
    case MEDIA_STARTED:
        if (!clocked) {
            /* The host stopped clocking - a pause, a source change. The
             * speaker is told rather than fed silence, so it can idle. */
            s_media = MEDIA_STOPPING;
            (void)esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_SUSPEND);
        }
        break;
    default: break; /* a request is in flight; its ack moves the state */
    }
}

static void a2dp_media_ack(esp_a2d_media_ctrl_t command, esp_a2d_media_ctrl_ack_t status)
{
    const bool ok = status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS;
    switch (s_media) {
    case MEDIA_CHECKING:
        if (command == ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY) {
            if (ok) {
                s_media = MEDIA_STARTING;
                (void)esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
            } else {
                s_media = MEDIA_IDLE;
            }
        }
        break;
    case MEDIA_STARTING:
        if (command == ESP_A2D_MEDIA_CTRL_START) {
            s_media = ok ? MEDIA_STARTED : MEDIA_IDLE;
            if (ok) {
                ESP_LOGI(TAG, "streaming to the speaker");
                if (s_listener.play != NULL) s_listener.play(JBT_PLAY_PLAYING);
            }
        }
        break;
    case MEDIA_STOPPING:
        if (command == ESP_A2D_MEDIA_CTRL_SUSPEND) {
            s_media = MEDIA_IDLE;
            ESP_LOGI(TAG, "stream suspended");
            if (s_listener.play != NULL) s_listener.play(JBT_PLAY_PAUSED);
        }
        break;
    default: break;
    }
}

static void a2dp_scan_result(esp_bt_gap_cb_param_t *param)
{
    a2dp_scan_result_t result = {0};
    memcpy(result.address, param->disc_res.bda, sizeof(result.address));
    result.rssi = -127;
    uint8_t *eir = NULL;
    for (int i = 0; i < param->disc_res.num_prop; ++i) {
        const esp_bt_gap_dev_prop_t *prop = &param->disc_res.prop[i];
        switch (prop->type) {
        case ESP_BT_GAP_DEV_PROP_COD: result.class_of_device = *(uint32_t *)prop->val; break;
        case ESP_BT_GAP_DEV_PROP_RSSI: result.rssi = *(int8_t *)prop->val; break;
        case ESP_BT_GAP_DEV_PROP_EIR: eir = (uint8_t *)prop->val; break;
        case ESP_BT_GAP_DEV_PROP_BDNAME:
            snprintf(result.name, sizeof(result.name), "%.*s", prop->len, (const char *)prop->val);
            break;
        default: break;
        }
    }
    /* Only things that render audio: a phone or a laptop answers an inquiry
     * too, and a list with them in it is a list to scroll past. */
    if (!esp_bt_gap_is_valid_cod(result.class_of_device) ||
        !(esp_bt_gap_get_cod_srvc(result.class_of_device) & ESP_BT_COD_SRVC_RENDERING)) {
        return;
    }
    if (result.name[0] == '\0' && eir != NULL) {
        uint8_t length = 0;
        uint8_t *name = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &length);
        if (name == NULL) name = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &length);
        if (name != NULL) snprintf(result.name, sizeof(result.name), "%.*s", length, (const char *)name);
    }
    ESP_LOGI(TAG, "found %02X:%02X:%02X:%02X:%02X:%02X %d dBm \"%s\"", result.address[0],
             result.address[1], result.address[2], result.address[3], result.address[4],
             result.address[5], result.rssi, result.name);
    if (s_listener.scan_result != NULL) s_listener.scan_result(&result);
}

static void a2dp_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT:
        if (s_scanning) a2dp_scan_result(param);
        break;
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        s_scanning = param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED;
        ESP_LOGI(TAG, "scan %s", s_scanning ? "started" : "ended");
        if (s_listener.scanning != NULL) s_listener.scanning(s_scanning);
        break;
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "paired with \"%s\"", param->auth_cmpl.device_name);
        } else {
            ESP_LOGW(TAG, "pairing failed: %d", param->auth_cmpl.stat);
        }
        break;
    case ESP_BT_GAP_PIN_REQ_EVT: {
        /* An old speaker with a fixed code: 0000 is what nearly all of them
         * use, and there is no keypad here to ask for another. */
        esp_bt_pin_code_t pin = {'0', '0', '0', '0'};
        (void)esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin);
        break;
    }
    case ESP_BT_GAP_READ_REMOTE_NAME_EVT:
        if (param->read_rmt_name.stat == ESP_BT_STATUS_SUCCESS && s_listener.peer_name != NULL) {
            s_listener.peer_name((const char *)param->read_rmt_name.rmt_name);
        }
        break;
    default: break;
    }
}

static void a2dp_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        const uint8_t *address = param->conn_stat.remote_bda;
        jbt_conn_t state = JBT_CONN_NONE;
        switch (param->conn_stat.state) {
        case ESP_A2D_CONNECTION_STATE_CONNECTING: state = JBT_CONN_CONNECTING; break;
        case ESP_A2D_CONNECTION_STATE_CONNECTED: state = JBT_CONN_CONNECTED; break;
        default: break;
        }
        s_connected = state == JBT_CONN_CONNECTED;
        if (s_connected) {
            memcpy(s_peer, address, sizeof(s_peer));
            s_media = MEDIA_IDLE;
            (void)esp_bt_gap_read_remote_name(s_peer);
        } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            memset(s_peer, 0, sizeof(s_peer));
            s_media = MEDIA_IDLE;
            s_peer_takes_volume = false;
        }
        ESP_LOGI(TAG, "connection %d %02X:%02X:%02X:%02X:%02X:%02X", param->conn_stat.state,
                 address[0], address[1], address[2], address[3], address[4], address[5]);
        if (s_listener.connection != NULL) s_listener.connection(state, address);
        break;
    }
    case ESP_A2D_MEDIA_CTRL_ACK_EVT:
        a2dp_media_ack(param->media_ctrl_stat.cmd, param->media_ctrl_stat.status);
        break;
    case ESP_A2D_AUDIO_STATE_EVT:
        ESP_LOGI(TAG, "audio %s", param->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED ? "started" : "stopped");
        break;
    case ESP_A2D_PROF_STATE_EVT:
        if (s_listener.profile != NULL) {
            s_listener.profile(param->a2d_prof_stat.init_state == ESP_A2D_INIT_SUCCESS);
        }
        break;
    case ESP_A2D_AUDIO_CFG_EVT:
    case ESP_A2D_REPORT_SNK_DELAY_VALUE_EVT:
        break;
    default:
        ESP_LOGD(TAG, "a2dp event %d", event);
        break;
    }
}

static void a2dp_subscribe_volume(void)
{
    (void)esp_avrc_ct_send_register_notification_cmd(a2dp_next_transaction(), ESP_AVRC_RN_VOLUME_CHANGE, 0U);
}

/* The speaker's own volume: we are the controller here, and a speaker that
 * supports absolute volume takes it as a command. */
static void a2dp_ct_callback(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
    switch (event) {
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT:
        s_peer_takes_volume = (param->rmt_feats.feat_mask & ESP_AVRC_FEAT_ADV_CTRL) != 0U;
        ESP_LOGI(TAG, "speaker %s absolute volume", s_peer_takes_volume ? "takes" : "does not take");
        break;
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
        if (param->conn_stat.connected) {
            (void)esp_avrc_ct_send_get_rn_capabilities_cmd(a2dp_next_transaction());
        }
        break;
    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT:
        /* A speaker with its own volume wheel does not send it as keys: it
         * changes the level itself and tells a subscribed controller. */
        if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST,
                                               (esp_avrc_rn_evt_cap_mask_t *)&param->get_rn_caps_rsp.evt_set,
                                               ESP_AVRC_RN_VOLUME_CHANGE)) {
            a2dp_subscribe_volume();
        }
        break;
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
        if (param->change_ntf.event_id == ESP_AVRC_RN_VOLUME_CHANGE) {
            const uint8_t volume = param->change_ntf.event_parameter.volume & 0x7FU;
            ESP_LOGI(TAG, "speaker turned its volume to %u", volume);
            if (s_listener.volume != NULL) s_listener.volume(volume);
            /* A notification fires once; ask again for the next turn. */
            a2dp_subscribe_volume();
        }
        break;
    case ESP_AVRC_CT_SET_ABSOLUTE_VOLUME_RSP_EVT:
        break;
    default:
        ESP_LOGD(TAG, "avrc ct event %d", event);
        break;
    }
}

/* The speaker's side of AVRCP: its buttons come here as passthrough, its
 * volume wheel as SetAbsoluteVolume. We are the "phone" to it. */
static void a2dp_tg_callback(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param)
{
    switch (event) {
    case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT: {
        if (param->psth_cmd.key_state != ESP_AVRC_PT_CMD_STATE_PRESSED) break;
        jbt_key_t key;
        switch (param->psth_cmd.key_code) {
        case ESP_AVRC_PT_CMD_PLAY: key = JBT_KEY_PLAY; break;
        case ESP_AVRC_PT_CMD_PAUSE: key = JBT_KEY_PAUSE; break;
        case ESP_AVRC_PT_CMD_STOP: key = JBT_KEY_STOP; break;
        case ESP_AVRC_PT_CMD_FORWARD: key = JBT_KEY_NEXT; break;
        case ESP_AVRC_PT_CMD_BACKWARD: key = JBT_KEY_PREV; break;
        case ESP_AVRC_PT_CMD_FAST_FORWARD: key = JBT_KEY_FAST_FORWARD; break;
        case ESP_AVRC_PT_CMD_REWIND: key = JBT_KEY_REWIND; break;
        case ESP_AVRC_PT_CMD_VOL_UP: key = JBT_KEY_VOLUME_UP; break;
        case ESP_AVRC_PT_CMD_VOL_DOWN: key = JBT_KEY_VOLUME_DOWN; break;
        case ESP_AVRC_PT_CMD_MUTE: key = JBT_KEY_MUTE; break;
        default: ESP_LOGI(TAG, "speaker key 0x%02x ignored", param->psth_cmd.key_code); return;
        }
        ESP_LOGI(TAG, "speaker key %d", key);
        if (s_listener.key != NULL) s_listener.key(key);
        break;
    }
    case ESP_AVRC_TG_CONNECTION_STATE_EVT:
        ESP_LOGI(TAG, "speaker's controller %s", param->conn_stat.connected ? "connected" : "gone");
        break;
    case ESP_AVRC_TG_REMOTE_FEATURES_EVT:
        ESP_LOGI(TAG, "speaker's controller features 0x%x flags 0x%x", (unsigned)param->rmt_feats.feat_mask,
                 (unsigned)param->rmt_feats.ct_feat_flag);
        break;
    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT:
        ESP_LOGI(TAG, "speaker set volume %u", param->set_abs_vol.volume & 0x7FU);
        if (s_listener.volume != NULL) s_listener.volume(param->set_abs_vol.volume & 0x7FU);
        break;
    case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT:
        if (param->reg_ntf.event_id == ESP_AVRC_RN_VOLUME_CHANGE) {
            /* The speaker may ask to hear our volume; answer with the
             * interim so the request does not hang, and leave it there. */
            esp_avrc_rn_param_t rn = {.volume = 100};
            (void)esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_INTERIM, &rn);
        }
        break;
    default: break;
    }
}

esp_err_t a2dp_source_start(const a2dp_source_listener_t *listener)
{
    if (listener == NULL) return ESP_ERR_INVALID_ARG;
    if (s_up) return ESP_OK;
    s_listener = *listener;
    if (s_media_timer == NULL) {
        const esp_timer_create_args_t tick = {.callback = a2dp_media_tick, .name = "media_tick"};
        ESP_RETURN_ON_ERROR(esp_timer_create(&tick, &s_media_timer), TAG, "timer");
    }
    ESP_RETURN_ON_ERROR(esp_bt_gap_register_callback(a2dp_gap_callback), TAG, "gap cb");
    ESP_RETURN_ON_ERROR(esp_avrc_ct_init(), TAG, "avrc ct");
    ESP_RETURN_ON_ERROR(esp_avrc_ct_register_callback(a2dp_ct_callback), TAG, "avrc ct cb");
    ESP_RETURN_ON_ERROR(esp_avrc_tg_init(), TAG, "avrc tg");
    ESP_RETURN_ON_ERROR(esp_avrc_tg_register_callback(a2dp_tg_callback), TAG, "avrc tg cb");
    /* The keys a speaker carries, accepted rather than refused: the default
     * filter is empty, and a refused key never reaches the callback. Listed
     * one by one because the set must stay inside the stack's allowed set -
     * one code outside it (RECORD, EJECT) and the whole call is rejected,
     * which is how the speaker's buttons once did nothing. Not read back
     * from the stack: the target's init is still in flight here. */
    static const esp_avrc_pt_cmd_t codes[] = {
        ESP_AVRC_PT_CMD_VOL_UP, ESP_AVRC_PT_CMD_VOL_DOWN, ESP_AVRC_PT_CMD_MUTE,
        ESP_AVRC_PT_CMD_PLAY, ESP_AVRC_PT_CMD_STOP, ESP_AVRC_PT_CMD_PAUSE,
        ESP_AVRC_PT_CMD_REWIND, ESP_AVRC_PT_CMD_FAST_FORWARD,
        ESP_AVRC_PT_CMD_FORWARD, ESP_AVRC_PT_CMD_BACKWARD,
    };
    esp_avrc_psth_bit_mask_t keys = {0};
    for (size_t i = 0; i < sizeof(codes) / sizeof(codes[0]); ++i) {
        esp_avrc_psth_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &keys, codes[i]);
    }
    ESP_RETURN_ON_ERROR(esp_avrc_tg_set_psth_cmd_filter(ESP_AVRC_PSTH_FILTER_SUPPORTED_CMD, &keys), TAG,
                        "psth supported");
    esp_avrc_rn_evt_cap_mask_t capabilities = {0};
    esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &capabilities, ESP_AVRC_RN_VOLUME_CHANGE);
    (void)esp_avrc_tg_set_rn_evt_cap(&capabilities);
    ESP_RETURN_ON_ERROR(esp_a2d_register_callback(a2dp_callback), TAG, "a2dp cb");
    ESP_RETURN_ON_ERROR(esp_a2d_source_register_data_callback(a2dp_data_callback), TAG, "data cb");
    ESP_RETURN_ON_ERROR(esp_a2d_source_init(), TAG, "source init");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(s_media_timer, A2DP_MEDIA_TICK_US), TAG, "tick");
    /* Not discoverable: nobody connects to a source. Connectable so a
     * speaker that was paired can come back on its own. */
    (void)esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    s_up = true;
    s_media = MEDIA_IDLE;
    ESP_LOGI(TAG, "ready");
    return ESP_OK;
}

esp_err_t a2dp_source_stop(void)
{
    if (!s_up) return ESP_OK;
    s_up = false;
    (void)esp_timer_stop(s_media_timer);
    if (s_scanning) (void)esp_bt_gap_cancel_discovery();
    if (s_connected) (void)esp_a2d_source_disconnect(s_peer);
    (void)esp_avrc_tg_deinit();
    (void)esp_avrc_ct_deinit();
    ESP_RETURN_ON_ERROR(esp_a2d_source_deinit(), TAG, "source deinit");
    return ESP_OK;
}

esp_err_t a2dp_source_scan(bool on)
{
    if (!s_up) return ESP_ERR_INVALID_STATE;
    if (on) {
        if (s_scanning) return ESP_OK;
        return esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, A2DP_SCAN_LENGTH, 0);
    }
    if (!s_scanning) return ESP_OK;
    return esp_bt_gap_cancel_discovery();
}

esp_err_t a2dp_source_connect(const uint8_t *address)
{
    if (!s_up) return ESP_ERR_INVALID_STATE;
    if (s_scanning) (void)esp_bt_gap_cancel_discovery();
    esp_bd_addr_t bda;
    memcpy(bda, address, sizeof(bda));
    ESP_LOGI(TAG, "connecting to %02X:%02X:%02X:%02X:%02X:%02X", bda[0], bda[1], bda[2], bda[3],
             bda[4], bda[5]);
    return esp_a2d_source_connect(bda);
}

esp_err_t a2dp_source_disconnect(void)
{
    if (!s_connected) return ESP_OK;
    return esp_a2d_source_disconnect(s_peer);
}

esp_err_t a2dp_source_set_volume(uint8_t volume)
{
    if (!s_connected || !s_peer_takes_volume) return ESP_ERR_NOT_SUPPORTED;
    return esp_avrc_ct_send_set_absolute_volume_cmd(a2dp_next_transaction(), volume > 127U ? 127U : volume);
}
