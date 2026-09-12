#pragma once

/* The UART link to the host: frames in, frames out, and the log mirrored.
 *
 * One receive task owns the UART and feeds the decoder; a complete frame is
 * handed to the handler on that task, so the handler must be quick - update
 * state, send a reply, post to a queue - and never block on Bluetooth. Sends
 * come from any task and are serialised by a mutex, with the sequence number
 * assigned inside so two tasks never reuse one.
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "jbt_proto.h"

typedef void (*jbt_link_handler_t)(const jbt_frame_t *frame, void *context);

/* Opens the UART named in menuconfig and starts the receive task. The
 * handler runs on that task for every good frame. */
esp_err_t jbt_link_start(jbt_link_handler_t handler, void *context);

/* Sends one frame; the sequence number is assigned here and returned through
 * `seq` when the caller wants to match an ACK. Blocks only while the UART's
 * transmit buffer is full, which a well-behaved host never lets happen for
 * long. */
esp_err_t jbt_link_send(uint8_t type, uint8_t flags, const uint8_t *payload, size_t length,
                        uint8_t *seq);

/* The ACK for a frame that asked for one. */
esp_err_t jbt_link_ack(uint8_t seq, jbt_result_t result);

/* A LOG frame with free text. Best effort: dropped, not queued, when the
 * transmit buffer has no room, because a log line must never stall audio. */
void jbt_link_log(uint8_t level, const char *format, ...) __attribute__((format(printf, 2, 3)));

/* Frames the decoder threw away since start - noise, bad CRC - for STATUS
 * and the PC tool. */
uint32_t jbt_link_dropped(void);
