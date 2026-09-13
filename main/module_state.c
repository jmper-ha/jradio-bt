#include "module_state.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"

#define MODULE_NVS_NAMESPACE "jbt"
#define MODULE_NVS_NAME_KEY "name"
#define MODULE_NVS_PEER_KEY "peer"
#define MODULE_NVS_SPEAKER_KEY "speaker"

static module_state_t s_state;
static SemaphoreHandle_t s_lock;

static void lock(void) { xSemaphoreTake(s_lock, portMAX_DELAY); }
static void unlock(void) { xSemaphoreGive(s_lock); }

esp_err_t module_state_init(const char *default_name)
{
    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) return ESP_ERR_NO_MEM;
    memset(&s_state, 0, sizeof(s_state));
    s_state.status.mode = JBT_MODE_OFF;
    s_state.status.volume = 100;

    nvs_handle_t nvs;
    size_t length = sizeof(s_state.name);
    if (nvs_open(MODULE_NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        if (nvs_get_str(nvs, MODULE_NVS_NAME_KEY, s_state.name, &length) != ESP_OK) {
            s_state.name[0] = '\0';
        }
        nvs_close(nvs);
    }
    if (s_state.name[0] == '\0') {
        snprintf(s_state.name, sizeof(s_state.name), "%s", default_name);
    }
    return ESP_OK;
}

void module_state_get(module_state_t *out)
{
    lock();
    *out = s_state;
    unlock();
}

bool module_state_set_name(const char *name)
{
    if (name == NULL || name[0] == '\0' || strlen(name) > MODULE_NAME_MAX) return false;
    lock();
    snprintf(s_state.name, sizeof(s_state.name), "%s", name);
    unlock();
    nvs_handle_t nvs;
    if (nvs_open(MODULE_NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        (void)nvs_set_str(nvs, MODULE_NVS_NAME_KEY, name);
        (void)nvs_commit(nvs);
        nvs_close(nvs);
    }
    return true;
}

void module_state_set_mode(jbt_mode_t mode)
{
    lock();
    s_state.status.mode = (uint8_t)mode;
    unlock();
}

void module_state_set_connection(jbt_conn_t connection, const uint8_t *peer, const char *peer_name)
{
    lock();
    s_state.status.connection = (uint8_t)connection;
    if (peer != NULL) {
        memcpy(s_state.status.peer, peer, sizeof(s_state.status.peer));
    } else {
        memset(s_state.status.peer, 0, sizeof(s_state.status.peer));
    }
    snprintf(s_state.peer_name, sizeof(s_state.peer_name), "%s", peer_name != NULL ? peer_name : "");
    unlock();
}

void module_state_set_play(jbt_play_t play)
{
    lock();
    s_state.status.play = (uint8_t)play;
    unlock();
}

void module_state_set_volume(uint8_t volume)
{
    lock();
    s_state.status.volume = volume > 127U ? 127U : volume;
    unlock();
}

void module_state_set_codec(jbt_codec_t codec, uint32_t sample_rate)
{
    lock();
    s_state.status.codec = (uint8_t)codec;
    s_state.status.sample_rate = sample_rate;
    unlock();
}

static bool nvs_address_get(const char *key, uint8_t out[6])
{
    nvs_handle_t nvs;
    if (nvs_open(MODULE_NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) return false;
    size_t length = 6U;
    const bool found = nvs_get_blob(nvs, key, out, &length) == ESP_OK && length == 6U;
    nvs_close(nvs);
    return found;
}

static void nvs_address_set(const char *key, const uint8_t peer[6])
{
    uint8_t known[6];
    if (nvs_address_get(key, known) && memcmp(known, peer, 6U) == 0) return;
    nvs_handle_t nvs;
    if (nvs_open(MODULE_NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        (void)nvs_set_blob(nvs, key, peer, 6U);
        (void)nvs_commit(nvs);
        nvs_close(nvs);
    }
}

static void nvs_address_erase(const char *key)
{
    nvs_handle_t nvs;
    if (nvs_open(MODULE_NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        (void)nvs_erase_key(nvs, key);
        (void)nvs_commit(nvs);
        nvs_close(nvs);
    }
}

bool module_state_last_peer(uint8_t out[6]) { return nvs_address_get(MODULE_NVS_PEER_KEY, out); }
void module_state_remember_peer(const uint8_t peer[6]) { nvs_address_set(MODULE_NVS_PEER_KEY, peer); }
void module_state_forget_peer(void) { nvs_address_erase(MODULE_NVS_PEER_KEY); }
bool module_state_last_speaker(uint8_t out[6]) { return nvs_address_get(MODULE_NVS_SPEAKER_KEY, out); }
void module_state_remember_speaker(const uint8_t peer[6]) { nvs_address_set(MODULE_NVS_SPEAKER_KEY, peer); }
void module_state_forget_speaker(void) { nvs_address_erase(MODULE_NVS_SPEAKER_KEY); }
