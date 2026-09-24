#pragma once

/* The I2S output: the module as master of the shared bus, in sink mode.
 *
 * The bus has three owners in turn - the host, this module, the DAC that
 * only listens - so "claim" and "release" are the two operations that
 * matter: claimed, the three pins are driven by this chip; released, they
 * are inputs and the host may drive them. The host and the module agree on
 * who owns the bus through SET_MODE/MODE_ACK; this file only does what it
 * is told, and does it in an order that never has both ends driving.
 *
 * PCM from the Bluetooth stack goes through a ring buffer: the decoder hands
 * out packets on its own schedule, the DAC wants a steady stream, and the
 * buffer is what turns the one into the other. It refills to a quarter
 * before it starts draining, so a late packet does not become a gap. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/* Creates the I2S channel on the pins from menuconfig and starts driving
 * them at the format last set (44.1 kHz stereo until told otherwise). */
esp_err_t audio_out_claim(void);

/* Stops, deletes the channel and turns the pins back into inputs. Safe when
 * not claimed. */
void audio_out_release(void);

bool audio_out_is_claimed(void);

/* The stream's format, from the codec negotiation. Reconfigures a claimed
 * channel in place; remembered for the next claim otherwise. */
esp_err_t audio_out_set_format(uint32_t sample_rate, uint8_t channels);

/* The stream started or was suspended: on suspend the buffer is emptied so
 * the next start begins on fresh audio, not on the tail of the last. */
void audio_out_stream(bool started);

/* Called from the Bluetooth decoder's task with 16-bit PCM. Returns how
 * much was taken; the rest is dropped - a full buffer means the DAC is
 * behind, and dropping here is better than stalling the stack. */
size_t audio_out_write(const uint8_t *pcm, size_t length);

/* 0..127, the AVRCP scale; applied to the samples on their way out. */
void audio_out_set_volume(uint8_t volume);

/* RMS per channel (0..32768) of what was clocked to the DAC since the previous
 * call, before the volume; false when nothing was played since. For the
 * host's level meter, sent as JBT_MSG_LEVEL. */
bool audio_out_level_take(uint16_t *left, uint16_t *right);
