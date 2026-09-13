#pragma once

/* The I2S input: the module as a slave on the shared bus, in source mode,
 * listening to what the host clocks out to the DAC.
 *
 * The host drives BCLK and LRCK; this end only reads DATA - the same wire
 * the DAC reads, so the DAC still plays and the module hears the same
 * bytes. The host says what rate it is clocking (I2S_FORMAT), and the A2DP
 * source wants 44.1 kHz stereo and nothing else (Bluedroid's feeding format
 * is fixed), so the samples are resampled here whenever the two differ - a
 * linear interpolation, which is enough for a radio and costs the WROOM a
 * few percent.
 *
 * Between the reader and the encoder sits a ring: the encoder asks for PCM
 * on its own schedule, in its own sizes, and the reader delivers DMA blocks
 * on the bus's. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/* Opens the channel as a slave on the pins from menuconfig and starts
 * reading. The bus may be silent at this point; reads simply time out
 * until the host clocks it. */
esp_err_t audio_in_open(void);

/* Stops reading and returns the pins to plain inputs. Safe when closed. */
void audio_in_close(void);

bool audio_in_is_open(void);

/* What the host is clocking. The reader reconfigures its slot width and
 * the resampler's ratio; the channel count is the host's, and mono is
 * doubled up for the encoder. */
esp_err_t audio_in_set_format(uint32_t sample_rate, uint8_t bits, uint8_t channels);

/* Fills `out` with 44.1 kHz 16-bit stereo PCM for the encoder: as much as
 * is buffered, up to `length` bytes, the rest zeroed - silence rather than
 * a stall, because the encoder's clock is the phone's and it must not
 * wait. Returns the bytes of real audio in the fill. */
size_t audio_in_read(uint8_t *out, size_t length);

/* Whether the bus is being clocked: false when no LRCK edges have arrived
 * for a while, which is how the host having stopped is noticed here. */
bool audio_in_clocked(void);
