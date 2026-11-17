#include "a2dp_sink.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "audio_out.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_bt_defs.h"
#include "esp_check.h"
#include "esp_gap_bt_api.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "a2dp_sink";

/* Everything the phone is asked for on a track change, the cover's handle
 * included: a phone without cover art simply leaves that attribute out. */
#define A2DP_METADATA_MASK                                                                        \
    (ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST | ESP_AVRC_MD_ATTR_ALBUM |                  \
     ESP_AVRC_MD_ATTR_GENRE | ESP_AVRC_MD_ATTR_PLAYING_TIME | ESP_AVRC_MD_ATTR_TRACK_NUM |         \
     ESP_AVRC_MD_ATTR_COVER_ART)
/* The most cover the module keeps: this chip has no PSRAM, and the linked
 * thumbnail a phone hands out is a 200x200 JPEG - 7 to 48 KB from Yandex
 * Music. A cover that would not fit is abandoned, and the track shows
 * without one. */
#define A2DP_COVER_MAX (48U * 1024U)
/* The picture is kept in pieces of this size, none of them next to another.
 * One buffer grown with realloc wanted the whole cover in one block, and a
 * second copy while it moved: after a few minutes of a phone playing the
 * largest block was 21-28 KB of some 55 KB free, and every cover past that
 * was dropped - pictures came and went with their size (2026-11-17). */
#define A2DP_COVER_STEP (4U * 1024U)
#define A2DP_COVER_PIECES (A2DP_COVER_MAX / A2DP_COVER_STEP)
#define A2DP_COVER_HANDLE_LEN 7U
/* How long after a track change the cover is fetched once more. An iPhone
 * playing Yandex Music named the previous track's handle in the new track's
 * metadata, and when no second description followed, the old picture stayed
 * on the screen for the whole track (2026-11-17). */
#define A2DP_COVER_RECHECK_US (3 * 1000 * 1000)
/* While the phone names no picture yet - an iPhone answers with an empty
 * handle until it has one - it is asked again this many times, a recheck
 * apart. */
#define A2DP_COVER_RECHECKS 5U
/* The attributes of one request arrive as separate events with nothing to
 * say which is the last - a phone leaves out what it does not know. So the
 * track is reported when they have stopped coming for this long. */
#define A2DP_METADATA_SETTLE_US (150 * 1000)
/* Position notifications, in seconds, when the phone supports them. */
#define A2DP_POSITION_INTERVAL_S 1U
/* How often the transport state is asked for while a phone is connected but
 * not believed to be playing: slower than the position, because nothing is
 * moving, and fast enough that a screen showing the wrong thing corrects
 * itself before anybody reaches for a button. */
#define A2DP_STATUS_INTERVAL_S 2U

static a2dp_sink_listener_t s_listener;
static bool s_enabled;
static bool s_connected;
static esp_bd_addr_t s_peer;
static uint8_t s_transaction;
static esp_avrc_rn_evt_cap_mask_t s_peer_capabilities;
static bool s_volume_notify_pending;
/* A local change the phone was not registered to hear: told to it the
 * moment it registers again. The phone re-registers 15-20 ms after each
 * "changed", and a knob turned fast lands several clicks in that gap. */
static bool s_volume_unsent;
static uint8_t s_volume = 100;
/* The last value the phone was told, and when. An iPhone moves its own
 * slider without saying so once it has heard a "changed" from us - the
 * next drag of it never reaches the sink - so while it is registered the
 * value is re-told once a second whenever it drifts from what it last
 * heard, and the slider snaps back to what the DAC is really at. */
static uint8_t s_volume_told = 0xFFU;
static esp_timer_handle_t s_volume_timer;
#define A2DP_VOLUME_RETELL_US (250 * 1000)

static a2dp_track_t s_track;
static SemaphoreHandle_t s_track_lock;
static esp_timer_handle_t s_settle_timer;
/* An iPhone offers no position notifications at all (its capability mask
 * lacks PLAY_POS_CHANGED), so while it plays the position is asked for once
 * a second instead. The answer carries the play state too, which is why the
 * one handler serves both. */
static esp_timer_handle_t s_poll_timer;
static bool s_playing;
/* Whether a stream has come since the connection. An iPhone that is
 * called back by the sink sometimes keeps its player paused, or streams
 * "to itself", until something presses play; after a few seconds without
 * a stream the sink presses it. Once per connection. */
static bool s_streamed;
static esp_timer_handle_t s_nudge_timer;
#define A2DP_NUDGE_US (5 * 1000 * 1000)
/* The cover-art channel (BIP over OBEX) and the one picture it fetched. */
static bool s_cover_channel;
static bool s_cover_fetching;
/* Whether this track's metadata named a picture at all. */
static bool s_cover_seen;
static uint8_t s_cover_handle[A2DP_COVER_HANDLE_LEN];
static uint8_t *s_cover[A2DP_COVER_PIECES];
static uint32_t s_cover_size;
static uint32_t s_cover_hash;
/* A cover dropped for want of memory, to be asked for again when the stream
 * starts. It arrives while the phone is still connecting, and the stack is
 * at its fullest then: on the bench a 24 KB cover met a largest block of
 * 14.8 KB, and five seconds later, with sound playing, it was 47 KB - but
 * the handle was not asked for again, and the track had no picture
 * (2026-11-16). */
static bool s_cover_short;
/* The phone said it serves covers. The channel can close under a connected
 * phone - the stack ran out of memory once and L2CAP dropped it - and was
 * opened only on connecting, so no track had a picture again until the
 * phone reconnected or the module was reset (2026-11-17). */
static bool s_cover_offered;
/* Why the next handle is fetched even when it is the one held: a new track
 * - the phone may keep one handle and change what is behind it - or the
 * recheck, which leaves the picture up while it fetches, so the same cover
 * does not blink. */
typedef enum {
    A2DP_COVER_REUSE = 0,
    A2DP_COVER_REFETCH_TRACK,
    A2DP_COVER_REFETCH_QUIET,
} a2dp_cover_refetch_t;
static volatile a2dp_cover_refetch_t s_cover_refetch;
/* A handle that came while another picture was being fetched, fetched next. */
static uint8_t s_cover_pending[A2DP_COVER_HANDLE_LEN];
static bool s_cover_pending_set;
static esp_timer_handle_t s_cover_recheck_timer;
static unsigned s_cover_rechecks;
/* The last request asked for the cover alone, from the recheck: the track
 * it settles is the one already told, and telling it again put each track
 * on the host four times while an iPhone readied its picture. */
static volatile bool s_cover_only_asked;
static SemaphoreHandle_t s_cover_lock;

static uint8_t a2dp_next_transaction(void)
{
    /* Transaction labels are four bits; the stack rejects a reuse in flight. */
    s_transaction = (uint8_t)((s_transaction + 1U) & 0x0FU);
    return s_transaction;
}

static void a2dp_scan_mode(void)
{
    /* Connectable only while enabled and free: a phone that already knows
     * us comes back on its own, which is what a speaker does; discoverable
     * is a separate, host-driven decision (PAIRING). */
    if (!s_enabled || s_connected) {
        (void)esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    } else {
        (void)esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    }
}

static void a2dp_cover_drop(void);

static void a2dp_settle_fired(void *arg)
{
    (void)arg;
    /* Static, not on the stack: this runs on the esp_timer task, whose
     * stack is small, and the listener below builds a frame on top of it -
     * a kilobyte of track here overflowed it on the first phone played. One
     * timer task, so one copy is never in use twice. */
    static a2dp_track_t copy;
    xSemaphoreTake(s_track_lock, portMAX_DELAY);
    copy = s_track;
    const bool cover_seen = s_cover_seen;
    xSemaphoreGive(s_track_lock);
    if (s_listener.track != NULL && !s_cover_only_asked) s_listener.track(&copy);
    /* Metadata that named no picture: this track has none, and the one
     * held is the previous track's. It may only have none yet, so the phone
     * is asked again a while. */
    if (!cover_seen) {
        a2dp_cover_drop();
        if (s_cover_channel && s_cover_rechecks > 0U && s_cover_rechecks < A2DP_COVER_RECHECKS) {
            (void)esp_timer_start_once(s_cover_recheck_timer, A2DP_COVER_RECHECK_US);
        }
    }
}

static void a2dp_poll_fired(void *arg)
{
    (void)arg;
    if (s_connected) (void)esp_avrc_ct_send_get_play_status_cmd(a2dp_next_transaction());
}

static void a2dp_nudge_fired(void *arg)
{
    (void)arg;
    if (!s_connected || s_streamed) return;
    ESP_LOGI(TAG, "no stream since the connection; pressing play");
    (void)esp_avrc_ct_send_passthrough_cmd(a2dp_next_transaction(), ESP_AVRC_PT_CMD_PLAY,
                                          ESP_AVRC_PT_CMD_STATE_PRESSED);
    (void)esp_avrc_ct_send_passthrough_cmd(a2dp_next_transaction(), ESP_AVRC_PT_CMD_PLAY,
                                          ESP_AVRC_PT_CMD_STATE_RELEASED);
}

static void a2dp_tell_volume(void)
{
    esp_avrc_rn_param_t rn = {.volume = s_volume};
    (void)esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_CHANGED, &rn);
    /* "Changed" closes the registration; the phone registers again. */
    s_volume_notify_pending = false;
    s_volume_unsent = false;
    s_volume_told = s_volume;
}

static void a2dp_volume_timer_fired(void *arg)
{
    (void)arg;
    if (s_connected && s_volume_notify_pending && (s_volume_unsent || s_volume_told != s_volume)) {
        a2dp_tell_volume();
    }
}

/* The poll keeps two things fresh, and used to run only while this module
 * already believed the phone was playing: the position, which only matters
 * then, and the transport state, which matters most when the belief is
 * wrong. A phone whose play status never arrives left that belief stuck at
 * "paused" with music coming out of the DAC. So it runs whenever a phone is
 * connected - one AVRCP command a second over a link that carries a song,
 * and the same rate the position always used - while the fast beat is still
 * reserved for the phone that cannot notify its position. */
static void a2dp_note_playing(bool playing)
{
    s_playing = playing;
    const bool notifies = esp_avrc_rn_evt_bit_mask_operation(
        ESP_AVRC_BIT_MASK_OP_TEST, &s_peer_capabilities, ESP_AVRC_RN_PLAY_POS_CHANGED);
    (void)esp_timer_stop(s_poll_timer);
    if (!s_connected) return;
    const uint32_t seconds = playing && !notifies ? A2DP_POSITION_INTERVAL_S
                                                  : A2DP_STATUS_INTERVAL_S;
    (void)esp_timer_start_periodic(s_poll_timer, (uint64_t)seconds * 1000000ULL);
}

static void a2dp_request_metadata(void)
{
    xSemaphoreTake(s_track_lock, portMAX_DELAY);
    memset(&s_track, 0, sizeof(s_track));
    s_cover_seen = false;
    xSemaphoreGive(s_track_lock);
    s_cover_refetch = A2DP_COVER_REFETCH_TRACK;
    s_cover_rechecks = 0U;
    s_cover_only_asked = false;
    (void)esp_timer_stop(s_cover_recheck_timer);
    (void)esp_timer_start_once(s_cover_recheck_timer, A2DP_COVER_RECHECK_US);
    (void)esp_avrc_ct_send_metadata_cmd(a2dp_next_transaction(), A2DP_METADATA_MASK);
}

static void a2dp_cover_recheck_fired(void *arg)
{
    (void)arg;
    if (!s_connected || !s_cover_channel) return;
    ++s_cover_rechecks;
    s_cover_refetch = A2DP_COVER_REFETCH_QUIET;
    s_cover_only_asked = true;
    (void)esp_avrc_ct_send_metadata_cmd(a2dp_next_transaction(), ESP_AVRC_MD_ATTR_COVER_ART);
}

static void a2dp_register_notifications(void)
{
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_peer_capabilities,
                                           ESP_AVRC_RN_TRACK_CHANGE)) {
        (void)esp_avrc_ct_send_register_notification_cmd(a2dp_next_transaction(),
                                                         ESP_AVRC_RN_TRACK_CHANGE, 0);
    }
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_peer_capabilities,
                                           ESP_AVRC_RN_PLAY_STATUS_CHANGE)) {
        (void)esp_avrc_ct_send_register_notification_cmd(a2dp_next_transaction(),
                                                         ESP_AVRC_RN_PLAY_STATUS_CHANGE, 0);
    }
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_peer_capabilities,
                                           ESP_AVRC_RN_PLAY_POS_CHANGED)) {
        (void)esp_avrc_ct_send_register_notification_cmd(a2dp_next_transaction(),
                                                         ESP_AVRC_RN_PLAY_POS_CHANGED,
                                                         A2DP_POSITION_INTERVAL_S);
    }
}

static jbt_play_t a2dp_play_state(esp_avrc_playback_stat_t status)
{
    switch (status) {
    case ESP_AVRC_PLAYBACK_PLAYING:
    case ESP_AVRC_PLAYBACK_FWD_SEEK:
    case ESP_AVRC_PLAYBACK_REV_SEEK: return JBT_PLAY_PLAYING;
    case ESP_AVRC_PLAYBACK_PAUSED: return JBT_PLAY_PAUSED;
    default: return JBT_PLAY_STOPPED;
    }
}

static void a2dp_notification(uint8_t event_id, const esp_avrc_rn_param_t *parameter)
{
    switch (event_id) {
    case ESP_AVRC_RN_TRACK_CHANGE:
        a2dp_request_metadata();
        /* One-shot: register again for the next change. */
        (void)esp_avrc_ct_send_register_notification_cmd(a2dp_next_transaction(),
                                                         ESP_AVRC_RN_TRACK_CHANGE, 0);
        break;
    case ESP_AVRC_RN_PLAY_STATUS_CHANGE: {
        const jbt_play_t play = a2dp_play_state(parameter->playback);
        a2dp_note_playing(play == JBT_PLAY_PLAYING);
        if (s_listener.play != NULL) s_listener.play(play);
        (void)esp_avrc_ct_send_register_notification_cmd(a2dp_next_transaction(),
                                                         ESP_AVRC_RN_PLAY_STATUS_CHANGE, 0);
        break;
    }
    case ESP_AVRC_RN_PLAY_POS_CHANGED:
        if (s_listener.position != NULL) s_listener.position(parameter->play_pos);
        (void)esp_avrc_ct_send_register_notification_cmd(a2dp_next_transaction(),
                                                         ESP_AVRC_RN_PLAY_POS_CHANGED,
                                                         A2DP_POSITION_INTERVAL_S);
        break;
    default: break;
    }
}

/* FNV-1a over the bytes, in a2dp_cover_hash(): the host caches decoded
 * covers by this, so a podcast whose every chapter carries the same picture
 * is decoded once. */
static jbt_image_t a2dp_cover_kind(void)
{
    const uint8_t *data = s_cover[0];
    if (data == NULL) return JBT_IMAGE_JPEG;
    if (s_cover_size >= 8U && data[0] == 0x89U && data[1] == 'P') return JBT_IMAGE_PNG;
    if (s_cover_size >= 2U && data[0] == 'B' && data[1] == 'M') return JBT_IMAGE_BMP;
    return JBT_IMAGE_JPEG;
}

static uint32_t a2dp_cover_hash(void)
{
    uint32_t hash = 2166136261U;
    for (uint32_t at = 0U; at < s_cover_size; at += A2DP_COVER_STEP) {
        const uint32_t left = s_cover_size - at;
        const uint8_t *piece = s_cover[at / A2DP_COVER_STEP];
        for (uint32_t i = 0U; i < (left < A2DP_COVER_STEP ? left : A2DP_COVER_STEP); ++i) {
            hash = (hash ^ piece[i]) * 16777619U;
        }
    }
    return hash;
}

static void a2dp_cover_free(void)
{
    for (size_t i = 0U; i < A2DP_COVER_PIECES; ++i) {
        free(s_cover[i]);
        s_cover[i] = NULL;
    }
    s_cover_size = 0U;
}

/* Appends to the pieces, taking a new one as each fills. */
static bool a2dp_cover_append(const uint8_t *data, uint32_t length)
{
    while (length > 0U) {
        const uint32_t index = s_cover_size / A2DP_COVER_STEP;
        const uint32_t at = s_cover_size % A2DP_COVER_STEP;
        if (s_cover[index] == NULL) {
            s_cover[index] = malloc(A2DP_COVER_STEP);
            if (s_cover[index] == NULL) return false;
        }
        const uint32_t room = A2DP_COVER_STEP - at;
        const uint32_t n = length < room ? length : room;
        memcpy(&s_cover[index][at], data, n);
        s_cover_size += n;
        data += n;
        length -= n;
    }
    return true;
}

static void a2dp_cover_drop_quietly(bool quiet)
{
    xSemaphoreTake(s_cover_lock, portMAX_DELAY);
    const bool had = !quiet && (s_cover[0] != NULL || s_cover_fetching);
    a2dp_cover_free();
    s_cover_hash = 0U;
    xSemaphoreGive(s_cover_lock);
    memset(s_cover_handle, 0, sizeof(s_cover_handle));
    s_cover_fetching = false;
    /* Told at once, before any new picture is fetched: the host otherwise
     * kept the last track's cover up until the next one arrived, and for
     * a track without one, for good. */
    if (had && s_listener.cover != NULL) s_listener.cover(0U, JBT_IMAGE_JPEG, 0U);
}

static void a2dp_cover_drop(void)
{
    s_cover_pending_set = false;
    a2dp_cover_drop_quietly(false);
}

/* A handle came with the metadata: fetch the picture when it is a new one.
 * The buffer is allocated at the first chunk, whole, and the fetch gives up
 * rather than growing past the cap. */
static void a2dp_cover_handle(const uint8_t *handle, int length)
{
    if (!s_cover_channel && s_cover_offered && s_connected) {
        /* Opened again; once it is, the channel asks for this handle. */
        ESP_LOGI(TAG, "cover art channel closed under the phone; opening it again");
        (void)esp_avrc_ct_cover_art_connect(ESP_AVRC_CA_MTU_MAX);
        return;
    }
    if (!s_cover_channel || length != (int)A2DP_COVER_HANDLE_LEN) return;
    const a2dp_cover_refetch_t refetch = s_cover_refetch;
    s_cover_refetch = A2DP_COVER_REUSE;
    ESP_LOGI(TAG, "cover handle %.7s%s", (const char *)handle,
             refetch == A2DP_COVER_REFETCH_TRACK   ? " (new track)"
             : refetch == A2DP_COVER_REFETCH_QUIET ? " (recheck)"
                                                   : "");
    if (s_cover_fetching) {
        memcpy(s_cover_pending, handle, A2DP_COVER_HANDLE_LEN);
        s_cover_pending_set = true;
        return;
    }
    if (refetch == A2DP_COVER_REUSE && memcmp(handle, s_cover_handle, A2DP_COVER_HANDLE_LEN) == 0 &&
        s_cover[0] != NULL) {
        /* The same picture again - a chapter change, or the phone repeating
         * itself - is told to the host again too, since it may have thrown
         * its copy away with the track. */
        if (s_listener.cover != NULL) {
            s_listener.cover(s_cover_size, a2dp_cover_kind(), s_cover_hash);
        }
        return;
    }
    s_cover_pending_set = false;
    /* The handle held, fetched again, keeps its picture up until the new
     * bytes replace it: an iPhone describes a track twice, and the screen
     * blanked between the two. */
    const bool same = memcmp(handle, s_cover_handle, A2DP_COVER_HANDLE_LEN) == 0 && s_cover[0] != NULL;
    a2dp_cover_drop_quietly(refetch == A2DP_COVER_REFETCH_QUIET || same);
    memcpy(s_cover_handle, handle, A2DP_COVER_HANDLE_LEN);
    if (esp_avrc_ct_cover_art_get_linked_thumbnail(s_cover_handle) == ESP_OK) {
        s_cover_fetching = true;
    }
}

static void a2dp_cover_data(const uint8_t *data, uint16_t length, bool final, bool ok)
{
    if (!s_cover_fetching) return;
    bool failed = !ok;
    xSemaphoreTake(s_cover_lock, portMAX_DELAY);
    if (!failed && length > 0U) {
        if (s_cover_size + length > A2DP_COVER_MAX) {
            ESP_LOGW(TAG, "cover larger than %u bytes; dropped", (unsigned)A2DP_COVER_MAX);
            failed = true;
        } else if (!a2dp_cover_append(data, length)) {
            ESP_LOGW(TAG, "no room for a cover past %u bytes (free %u); dropped",
                     (unsigned)s_cover_size,
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT));
            failed = true;
            s_cover_short = true;
        }
    }
    if (failed) a2dp_cover_free();
    if (final && !failed) s_cover_hash = a2dp_cover_hash();
    const uint32_t size = s_cover_size;
    const uint32_t hash = s_cover_hash;
    const jbt_image_t kind = a2dp_cover_kind();
    xSemaphoreGive(s_cover_lock);
    if (failed || final) {
        s_cover_fetching = false;
        if (failed) {
            /* Not tried again for this handle: the same request would fail
             * the same way. The next track's handle starts afresh. */
            memset(s_cover_handle, 0, sizeof(s_cover_handle));
        } else {
            ESP_LOGI(TAG, "cover %u bytes, hash %08x", (unsigned)size, (unsigned)hash);
        }
        if (s_listener.cover != NULL) s_listener.cover(failed ? 0U : size, kind, failed ? 0U : hash);
        /* What came while this one was fetched is the newer word. */
        if (s_cover_pending_set) {
            s_cover_pending_set = false;
            s_cover_refetch = A2DP_COVER_REFETCH_QUIET;
            a2dp_cover_handle(s_cover_pending, (int)A2DP_COVER_HANDLE_LEN);
        }
    }
}

size_t a2dp_sink_cover_read(uint32_t offset, uint8_t *out, size_t max, uint32_t *hash)
{
    if (s_cover_lock == NULL) return 0U;
    xSemaphoreTake(s_cover_lock, portMAX_DELAY);
    size_t n = 0U;
    if (s_cover_hash != 0U && offset < s_cover_size) {
        n = s_cover_size - offset;
        if (n > max) n = max;
        // The pieces are apart, so a read across a seam is two copies.
        for (size_t done = 0U; done < n;) {
            const uint32_t at = offset + (uint32_t)done;
            const uint32_t in_piece = A2DP_COVER_STEP - at % A2DP_COVER_STEP;
            const size_t take = n - done < in_piece ? n - done : in_piece;
            memcpy(&out[done], &s_cover[at / A2DP_COVER_STEP][at % A2DP_COVER_STEP], take);
            done += take;
        }
    }
    if (hash != NULL) *hash = s_cover_hash;
    xSemaphoreGive(s_cover_lock);
    return n;
}

static void a2dp_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "paired with \"%s\"", param->auth_cmpl.device_name);
        } else {
            ESP_LOGW(TAG, "pairing failed: %d", param->auth_cmpl.stat);
        }
        break;
    case ESP_BT_GAP_READ_REMOTE_NAME_EVT:
        if (param->read_rmt_name.stat == ESP_BT_STATUS_SUCCESS && s_listener.peer_name != NULL) {
            s_listener.peer_name((const char *)param->read_rmt_name.rmt_name);
        }
        break;
    case ESP_BT_GAP_MODE_CHG_EVT:
    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
    case ESP_BT_GAP_ACL_PKT_TYPE_CHANGED_EVT:
    case ESP_BT_GAP_ENC_CHG_EVT:
        break;
    default:
        ESP_LOGD(TAG, "gap event %d", event);
        break;
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
            (void)esp_bt_gap_read_remote_name(s_peer);
            s_streamed = false;
            (void)esp_timer_stop(s_nudge_timer);
            (void)esp_timer_start_once(s_nudge_timer, A2DP_NUDGE_US);
            /* The poll wants to run from the connection, not from the first
             * thing the phone happens to say. */
            a2dp_note_playing(false);
        } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            audio_out_stream(false);
            a2dp_note_playing(false);
            memset(s_peer, 0, sizeof(s_peer));
            s_cover_channel = false;
            s_cover_offered = false;
            a2dp_cover_drop();
        }
        ESP_LOGI(TAG, "connection %d %02X:%02X:%02X:%02X:%02X:%02X", param->conn_stat.state,
                 address[0], address[1], address[2], address[3], address[4], address[5]);
        a2dp_scan_mode();
        if (s_listener.connection != NULL) s_listener.connection(state, address);
        break;
    }
    case ESP_A2D_AUDIO_STATE_EVT: {
        const bool started = param->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED;
        if (started) s_streamed = true;
        audio_out_stream(started);
        /* The memory beside it, because the audio ring is the one buffer here
         * worth growing and there is no PSRAM to grow it into: this is the
         * moment the stack has taken all it will for a playing stream. */
        ESP_LOGI(TAG, "audio %s; internal free %u, largest %u", started ? "started" : "suspended",
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
        /* And this is the transport state, reported as such. The comment here
         * used to claim AVRCP covered it - it does not: AVRCP tells us only
         * when it notifies, and a phone that starts playing without a
         * PLAY_STATUS_CHANGE left the board showing a pause icon over music
         * that was playing, with nothing able to clear it (a play key sent to
         * an already-playing phone changes nothing). A stream that is running
         * is sound leaving the DAC, which is what the state means; AVRCP
         * still corrects the detail, and is asked to right now. */
        a2dp_note_playing(started);
        if (s_listener.play != NULL) s_listener.play(started ? JBT_PLAY_PLAYING : JBT_PLAY_PAUSED);
        if (s_connected) (void)esp_avrc_ct_send_get_play_status_cmd(a2dp_next_transaction());
        /* The handle of what is playing, as when the channel opens: the
         * failed fetch forgot it, so it is fetched afresh. */
        if (started && s_cover_short && s_cover_channel && !s_cover_fetching) {
            s_cover_short = false;
            ESP_LOGI(TAG, "asking for the cover again, now that the stream has started");
            (void)esp_avrc_ct_send_metadata_cmd(a2dp_next_transaction(), ESP_AVRC_MD_ATTR_COVER_ART);
        }
        break;
    }
    case ESP_A2D_AUDIO_CFG_EVT: {
        const esp_a2d_mcc_t *mcc = &param->audio_cfg.mcc;
        uint32_t rate = 44100U;
        uint8_t channels = 2U;
        jbt_codec_t codec = JBT_CODEC_SBC;
        if (mcc->type == ESP_A2D_MCT_SBC) {
            const uint8_t freq = mcc->cie.sbc_info.samp_freq;
            if (freq & ESP_A2D_SBC_CIE_SF_48K) rate = 48000U;
            else if (freq & ESP_A2D_SBC_CIE_SF_44K) rate = 44100U;
            else if (freq & ESP_A2D_SBC_CIE_SF_32K) rate = 32000U;
            else if (freq & ESP_A2D_SBC_CIE_SF_16K) rate = 16000U;
            if (mcc->cie.sbc_info.ch_mode & ESP_A2D_SBC_CIE_CH_MODE_MONO) channels = 1U;
        } else {
            codec = JBT_CODEC_AAC;
        }
        (void)audio_out_set_format(rate, channels);
        ESP_LOGI(TAG, "codec %d, %u Hz, %u ch", mcc->type, (unsigned)rate, channels);
        if (s_listener.audio_format != NULL) s_listener.audio_format(codec, rate);
        break;
    }
    case ESP_A2D_PROF_STATE_EVT:
        if (s_listener.profile != NULL) {
            s_listener.profile(param->a2d_prof_stat.init_state == ESP_A2D_INIT_SUCCESS);
        }
        break;
    case ESP_A2D_SNK_PSC_CFG_EVT:
    case ESP_A2D_SNK_SET_DELAY_VALUE_EVT:
    case ESP_A2D_SNK_GET_DELAY_VALUE_EVT:
        break;
    default:
        ESP_LOGD(TAG, "a2dp event %d", event);
        break;
    }
}

static void a2dp_data_callback(const uint8_t *data, uint32_t length)
{
    (void)audio_out_write(data, length);
}

static void a2dp_ct_callback(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
    switch (event) {
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
        if (param->conn_stat.connected) {
            (void)esp_avrc_ct_send_get_rn_capabilities_cmd(a2dp_next_transaction());
        } else {
            s_peer_capabilities.bits = 0;
        }
        break;
    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT:
        s_peer_capabilities.bits = param->get_rn_caps_rsp.evt_set.bits;
        ESP_LOGI(TAG, "peer notifications 0x%04x", (unsigned)s_peer_capabilities.bits);
        a2dp_register_notifications();
        a2dp_request_metadata();
        (void)esp_avrc_ct_send_get_play_status_cmd(a2dp_next_transaction());
        break;
    case ESP_AVRC_CT_METADATA_RSP_EVT: {
        const int length = param->meta_rsp.attr_length;
        const char *text = (const char *)param->meta_rsp.attr_text;
        ESP_LOGD(TAG, "metadata attr 0x%02x, %d bytes", param->meta_rsp.attr_id, length);
        if (text == NULL || length < 0) break;
        char *field = NULL;
        xSemaphoreTake(s_track_lock, portMAX_DELAY);
        switch (param->meta_rsp.attr_id) {
        case ESP_AVRC_MD_ATTR_TITLE: field = s_track.title; break;
        case ESP_AVRC_MD_ATTR_ARTIST: field = s_track.artist; break;
        case ESP_AVRC_MD_ATTR_ALBUM: field = s_track.album; break;
        case ESP_AVRC_MD_ATTR_GENRE: field = s_track.genre; break;
        case ESP_AVRC_MD_ATTR_PLAYING_TIME: s_track.duration_ms = (uint32_t)strtoul(text, NULL, 10); break;
        case ESP_AVRC_MD_ATTR_TRACK_NUM: s_track.track_no = (uint32_t)strtoul(text, NULL, 10); break;
        case ESP_AVRC_MD_ATTR_COVER_ART:
            /* An iPhone names the attribute with nothing in it while the
             * picture is not ready; that kept the last track's cover up. */
            if (length != (int)A2DP_COVER_HANDLE_LEN) {
                ESP_LOGI(TAG, "no cover handle yet (%d bytes)", length);
                break;
            }
            s_cover_seen = true;
            xSemaphoreGive(s_track_lock);
            a2dp_cover_handle((const uint8_t *)text, length);
            xSemaphoreTake(s_track_lock, portMAX_DELAY);
            break;
        default: break;
        }
        if (field != NULL) {
            const size_t n = (size_t)length < A2DP_TEXT_MAX ? (size_t)length : A2DP_TEXT_MAX;
            memcpy(field, text, n);
            field[n] = '\0';
        }
        xSemaphoreGive(s_track_lock);
        (void)esp_timer_stop(s_settle_timer);
        (void)esp_timer_start_once(s_settle_timer, A2DP_METADATA_SETTLE_US);
        break;
    }
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
        ESP_LOGD(TAG, "notification %d", param->change_ntf.event_id);
        a2dp_notification(param->change_ntf.event_id, &param->change_ntf.event_parameter);
        break;
    case ESP_AVRC_CT_PLAY_STATUS_RSP_EVT: {
        const jbt_play_t play = a2dp_play_state(param->play_status_rsp.play_status);
        if (play != (s_playing ? JBT_PLAY_PLAYING : JBT_PLAY_STOPPED)) a2dp_note_playing(play == JBT_PLAY_PLAYING);
        if (s_listener.play != NULL) s_listener.play(play);
        if (s_listener.position != NULL) s_listener.position(param->play_status_rsp.song_position);
        break;
    }
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT:
        if (param->rmt_feats.tg_feat_flag & ESP_AVRC_FEAT_FLAG_TG_COVER_ART) {
            ESP_LOGI(TAG, "phone offers cover art");
            s_cover_offered = true;
            (void)esp_avrc_ct_cover_art_connect(ESP_AVRC_CA_MTU_MAX);
        }
        break;
    case ESP_AVRC_CT_COVER_ART_STATE_EVT:
        s_cover_channel = param->cover_art_state.state == ESP_AVRC_COVER_ART_CONNECTED;
        ESP_LOGI(TAG, "cover art channel %s", s_cover_channel ? "open" : "closed");
        if (s_cover_channel) {
            /* The handle of what is playing now, since the one that came
             * with the track's metadata arrived before the channel was up. */
            (void)esp_avrc_ct_send_metadata_cmd(a2dp_next_transaction(), ESP_AVRC_MD_ATTR_COVER_ART);
        } else {
            s_cover_fetching = false;
        }
        break;
    case ESP_AVRC_CT_COVER_ART_DATA_EVT:
        a2dp_cover_data(param->cover_art_data.p_data, param->cover_art_data.data_len,
                        param->cover_art_data.final,
                        param->cover_art_data.status == ESP_BT_STATUS_SUCCESS);
        break;
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT:
    case ESP_AVRC_CT_SET_ABSOLUTE_VOLUME_RSP_EVT:
        break;
    default:
        ESP_LOGD(TAG, "avrc ct event %d", event);
        break;
    }
}

static void a2dp_tg_callback(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param)
{
    switch (event) {
    case ESP_AVRC_TG_CONNECTION_STATE_EVT:
        s_volume_notify_pending = false;
        break;
    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT: {
        const uint8_t asked = param->set_abs_vol.volume & 0x7FU;
        s_volume = asked;
        s_volume_told = asked;
        audio_out_set_volume(s_volume);
        ESP_LOGD(TAG, "phone set volume %u", s_volume);
        if (s_listener.volume != NULL) s_listener.volume(s_volume);
        break;
    }
    case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT:
        if (param->reg_ntf.event_id == ESP_AVRC_RN_VOLUME_CHANGE) {
            /* The phone wants to hear about local changes: answer now with
             * "interim" and remember to send "changed" when the knob turns. */
            esp_avrc_rn_param_t rn = {.volume = s_volume};
            (void)esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_INTERIM, &rn);
            s_volume_notify_pending = true;
            /* The interim answer carries the current value; a change since
             * the last "changed" goes out on the timer's next tick, so the
             * phone is never told twice inside one registration. */
        }
        break;
    case ESP_AVRC_TG_REMOTE_FEATURES_EVT:
    case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT:
        break;
    default:
        ESP_LOGD(TAG, "avrc tg event %d", event);
        break;
    }
}

static bool s_prepared;
static bool s_profile_up;

/* The parts that live for the whole boot: locks, timers, the pairing
 * policy. Done once; the profile itself goes up and down around them. */
static esp_err_t a2dp_sink_prepare(void)
{
    if (s_prepared) return ESP_OK;
    s_track_lock = xSemaphoreCreateMutex();
    s_cover_lock = xSemaphoreCreateMutex();
    if (s_track_lock == NULL || s_cover_lock == NULL) return ESP_ERR_NO_MEM;
    const esp_timer_create_args_t settle = {.callback = a2dp_settle_fired, .name = "md_settle"};
    ESP_RETURN_ON_ERROR(esp_timer_create(&settle, &s_settle_timer), TAG, "timer");
    const esp_timer_create_args_t poll = {.callback = a2dp_poll_fired, .name = "pos_poll"};
    ESP_RETURN_ON_ERROR(esp_timer_create(&poll, &s_poll_timer), TAG, "poll timer");
    const esp_timer_create_args_t nudge = {.callback = a2dp_nudge_fired, .name = "nudge"};
    ESP_RETURN_ON_ERROR(esp_timer_create(&nudge, &s_nudge_timer), TAG, "nudge timer");
    const esp_timer_create_args_t recheck = {.callback = a2dp_cover_recheck_fired,
                                             .name = "cover_recheck"};
    ESP_RETURN_ON_ERROR(esp_timer_create(&recheck, &s_cover_recheck_timer), TAG,
                        "cover recheck timer");
    const esp_timer_create_args_t retell = {.callback = a2dp_volume_timer_fired, .name = "vol_retell"};
    ESP_RETURN_ON_ERROR(esp_timer_create(&retell, &s_volume_timer), TAG, "volume timer");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(s_volume_timer, A2DP_VOLUME_RETELL_US), TAG,
                        "volume timer start");

    /* "Just works" pairing: no display, no keyboard, like any speaker. */
    esp_bt_io_cap_t io_capability = ESP_BT_IO_CAP_NONE;
    ESP_RETURN_ON_ERROR(esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &io_capability,
                                                      sizeof(io_capability)),
                        TAG, "io cap");
    s_prepared = true;
    return ESP_OK;
}

esp_err_t a2dp_sink_start(const a2dp_sink_listener_t *listener)
{
    if (listener == NULL) return ESP_ERR_INVALID_ARG;
    s_listener = *listener;
    ESP_RETURN_ON_ERROR(a2dp_sink_prepare(), TAG, "prepare");
    if (s_profile_up) return ESP_OK;
    ESP_RETURN_ON_ERROR(esp_bt_gap_register_callback(a2dp_gap_callback), TAG, "gap cb");

    ESP_RETURN_ON_ERROR(esp_avrc_ct_init(), TAG, "avrc ct");
    ESP_RETURN_ON_ERROR(esp_avrc_ct_register_callback(a2dp_ct_callback), TAG, "avrc ct cb");
    ESP_RETURN_ON_ERROR(esp_avrc_tg_init(), TAG, "avrc tg");
    ESP_RETURN_ON_ERROR(esp_avrc_tg_register_callback(a2dp_tg_callback), TAG, "avrc tg cb");
    /* Tell the phone we take absolute volume, so its slider drives ours. */
    esp_avrc_rn_evt_cap_mask_t capabilities = {0};
    esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &capabilities,
                                       ESP_AVRC_RN_VOLUME_CHANGE);
    ESP_RETURN_ON_ERROR(esp_avrc_tg_set_rn_evt_cap(&capabilities), TAG, "avrc tg cap");

    ESP_RETURN_ON_ERROR(esp_a2d_register_callback(a2dp_callback), TAG, "a2dp cb");
    ESP_RETURN_ON_ERROR(esp_a2d_sink_register_data_callback(a2dp_data_callback), TAG, "a2dp data cb");
    ESP_RETURN_ON_ERROR(esp_a2d_sink_init(), TAG, "a2dp sink");
    audio_out_set_volume(s_volume);
    s_profile_up = true;
    a2dp_scan_mode();
    ESP_LOGI(TAG, "ready");
    return ESP_OK;
}

esp_err_t a2dp_sink_stop(void)
{
    if (!s_profile_up) return ESP_OK;
    s_profile_up = false;
    s_enabled = false;
    if (s_connected) (void)esp_a2d_sink_disconnect(s_peer);
    (void)esp_timer_stop(s_nudge_timer);
    (void)esp_timer_stop(s_poll_timer);
    (void)esp_avrc_tg_deinit();
    (void)esp_avrc_ct_deinit();
    ESP_RETURN_ON_ERROR(esp_a2d_sink_deinit(), TAG, "a2dp sink deinit");
    ESP_LOGI(TAG, "down");
    return ESP_OK;
}

void a2dp_sink_set_enabled(bool enabled)
{
    s_enabled = enabled;
    if (!enabled && s_connected) (void)esp_a2d_sink_disconnect(s_peer);
    a2dp_scan_mode();
}

esp_err_t a2dp_sink_connect(const uint8_t *address)
{
    if (!s_enabled) return ESP_ERR_INVALID_STATE;
    esp_bd_addr_t bda;
    memcpy(bda, address, sizeof(bda));
    return esp_a2d_sink_connect(bda);
}

esp_err_t a2dp_sink_disconnect(void)
{
    if (!s_connected) return ESP_OK;
    return esp_a2d_sink_disconnect(s_peer);
}

esp_err_t a2dp_sink_passthrough(jbt_key_t key)
{
    if (!s_connected) return ESP_ERR_INVALID_STATE;
    uint8_t code;
    switch (key) {
    case JBT_KEY_PLAY: code = ESP_AVRC_PT_CMD_PLAY; break;
    case JBT_KEY_PAUSE: code = ESP_AVRC_PT_CMD_PAUSE; break;
    case JBT_KEY_STOP: code = ESP_AVRC_PT_CMD_STOP; break;
    case JBT_KEY_NEXT: code = ESP_AVRC_PT_CMD_FORWARD; break;
    case JBT_KEY_PREV: code = ESP_AVRC_PT_CMD_BACKWARD; break;
    case JBT_KEY_FAST_FORWARD: code = ESP_AVRC_PT_CMD_FAST_FORWARD; break;
    case JBT_KEY_REWIND: code = ESP_AVRC_PT_CMD_REWIND; break;
    default: return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(esp_avrc_ct_send_passthrough_cmd(a2dp_next_transaction(), code,
                                                         ESP_AVRC_PT_CMD_STATE_PRESSED),
                        TAG, "key press");
    return esp_avrc_ct_send_passthrough_cmd(a2dp_next_transaction(), code,
                                            ESP_AVRC_PT_CMD_STATE_RELEASED);
}

void a2dp_sink_set_volume(uint8_t volume)
{
    const uint8_t wanted = volume > 127U ? 127U : volume;
    if (wanted == s_volume) return;
    s_volume = wanted;
    audio_out_set_volume(s_volume);
    /* Not told at once: a knob turned fast would send a "changed" per click,
     * and each closes the registration the phone then reopens - clicks in
     * that window were refused by the stack ("Event id not registered") and
     * the phone answered the ones that got through with its own SetVolume,
     * which read as the slider jumping back. The timer tells the phone the
     * latest value at most once a cycle, on a registration it holds. */
    s_volume_unsent = s_connected;
}
