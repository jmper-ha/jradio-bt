#pragma once

/* What the module knows about itself, the one copy that STATUS reports.
 * Written by whichever task learns something - the link task on a command,
 * the Bluetooth callbacks on a connection - and read as a snapshot, the way
 * jRadio's player_control hands out its own. */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "jbt_proto.h"

#define MODULE_NAME_MAX 32U
#define MODULE_PEER_NAME_MAX 64U

typedef struct {
    jbt_status_t status;
    char name[MODULE_NAME_MAX + 1U];
    char peer_name[MODULE_PEER_NAME_MAX + 1U];
} module_state_t;

/* Loads the name from NVS (or takes the default) and sets the rest to "off,
 * nothing connected". */
esp_err_t module_state_init(const char *default_name);

void module_state_get(module_state_t *out);

/* Stores the name in NVS as well; false when it does not fit. */
bool module_state_set_name(const char *name);

void module_state_set_mode(jbt_mode_t mode);
void module_state_set_connection(jbt_conn_t connection, const uint8_t *peer, const char *peer_name);
void module_state_set_play(jbt_play_t play);
void module_state_set_volume(uint8_t volume);
void module_state_set_codec(jbt_codec_t codec, uint32_t sample_rate);

/* The last phone that connected, kept in NVS so the module can go back to
 * it after a reboot: an iPhone does not come back to a sink on its own.
 * False when none is known. */
bool module_state_last_peer(uint8_t out[6]);
void module_state_remember_peer(const uint8_t peer[6]);
void module_state_forget_peer(void);
/* The same for the speaker the module last sent to. */
bool module_state_last_speaker(uint8_t out[6]);
void module_state_remember_speaker(const uint8_t peer[6]);
void module_state_forget_speaker(void);
