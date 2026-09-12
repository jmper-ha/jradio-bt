#include "jbt_link.h"

#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "jbt_link";

#define JBT_UART ((uart_port_t)CONFIG_JBT_UART_NUM)
/* The receive ring holds a few frames' worth so a burst from the host while
 * the task is busy is not lost; the transmit ring holds one cover chunk plus
 * the status traffic around it. */
#define JBT_RX_RING 4096
#define JBT_TX_RING 4096
/* The longest log line the mirror forwards; longer ones are clipped. Sized
 * to one TLV record, which is the most a LOG frame carries. */
#define JBT_LOG_LINE_MAX 200

static jbt_link_handler_t s_handler;
static void *s_context;
static jbt_decoder_t s_decoder;
static SemaphoreHandle_t s_send_lock;
static uint8_t s_next_seq;
static bool s_started;
#if CONFIG_JBT_LOG_MIRROR
static vprintf_like_t s_previous_vprintf;
#endif

/* The whole frame is written in one uart_write_bytes: the driver copies it
 * into its ring, so two senders interleaving bytes is impossible even
 * without the lock - the lock is for the sequence number and for keeping
 * one sender's frames in the order it sent them. */
static esp_err_t jbt_link_write(const jbt_frame_t *frame)
{
    uint8_t wire[JBT_WIRE_MAX];
    const size_t n = jbt_frame_encode(frame, wire, sizeof(wire));
    if (n == 0U) return ESP_ERR_INVALID_SIZE;
    const int written = uart_write_bytes(JBT_UART, wire, n);
    return written == (int)n ? ESP_OK : ESP_FAIL;
}

esp_err_t jbt_link_send(uint8_t type, uint8_t flags, const uint8_t *payload, size_t length,
                        uint8_t *seq)
{
    if (!s_started) return ESP_ERR_INVALID_STATE;
    if (length > JBT_PAYLOAD_MAX) return ESP_ERR_INVALID_SIZE;
    xSemaphoreTake(s_send_lock, portMAX_DELAY);
    const jbt_frame_t frame = {
        .type = type, .flags = flags, .seq = s_next_seq++, .len = (uint16_t)length,
        .payload = payload,
    };
    if (seq != NULL) *seq = frame.seq;
    const esp_err_t err = jbt_link_write(&frame);
    xSemaphoreGive(s_send_lock);
    return err;
}

esp_err_t jbt_link_ack(uint8_t seq, jbt_result_t result)
{
    const uint8_t payload[2] = {seq, (uint8_t)result};
    return jbt_link_send(JBT_MSG_ACK, JBT_FLAG_IS_ACK, payload, sizeof(payload), NULL);
}

static void jbt_link_log_line(uint8_t level, const char *text, size_t length)
{
    if (!s_started) return;
    /* Best effort: when the ring cannot take the whole frame right now, the
     * line is dropped rather than the caller stalled. Escaping can at most
     * double the bytes, and the check is on that worst case. */
    const size_t needed = 2U * (JBT_HEADER_SIZE + 3U + length + JBT_CRC_SIZE) + 2U;
    size_t free_space = 0U;
    if (uart_get_tx_buffer_free_size(JBT_UART, &free_space) != ESP_OK || free_space < needed) {
        return;
    }
    uint8_t payload[3U + JBT_LOG_LINE_MAX];
    jbt_writer_t writer;
    jbt_writer_init(&writer, payload, sizeof(payload));
    jbt_put_u8(&writer, level);
    jbt_put_tlv_bytes(&writer, JBT_TAG_TEXT, text, length);
    if (writer.overflow) return;
    (void)jbt_link_send(JBT_MSG_LOG, 0U, payload, writer.length, NULL);
}

void jbt_link_log(uint8_t level, const char *format, ...)
{
    char line[JBT_LOG_LINE_MAX + 1U];
    va_list args;
    va_start(args, format);
    int n = vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    if (n < 0) return;
    if ((size_t)n > JBT_LOG_LINE_MAX) n = JBT_LOG_LINE_MAX;
    jbt_link_log_line(level, line, (size_t)n);
}

#if CONFIG_JBT_LOG_MIRROR
/* Every ESP_LOG line, formatted once for the console and once more for the
 * host. The level is read back off the line's first letter, which is what
 * the default format puts there; the colour escapes are stripped. Re-entry
 * is guarded because the UART driver logs too. */
static int jbt_link_vprintf(const char *format, va_list args)
{
    static volatile bool inside;
    va_list copy;
    va_copy(copy, args);
    const int printed = s_previous_vprintf(format, args);
    if (!inside && s_started) {
        inside = true;
        char line[JBT_LOG_LINE_MAX + 1U];
        int n = vsnprintf(line, sizeof(line), format, copy);
        if (n > 0) {
            if ((size_t)n > JBT_LOG_LINE_MAX) n = JBT_LOG_LINE_MAX;
            const char *text = line;
            size_t length = (size_t)n;
            /* "\033[0;32mI (1234) tag: text\033[0m\n" -> "I (1234) tag: text" */
            if (text[0] == '\033') {
                const char *m = strchr(text, 'm');
                if (m != NULL) {
                    length -= (size_t)(m + 1 - text);
                    text = m + 1;
                }
            }
            /* The trailing reset sequence, then the newline. */
            const char *reset = strstr(text, "\033[0m");
            if (reset != NULL) length = (size_t)(reset - text);
            while (length > 0U && (text[length - 1U] == '\n' || text[length - 1U] == '\r')) --length;
            uint8_t level = ESP_LOG_INFO;
            switch (text[0]) {
            case 'E': level = ESP_LOG_ERROR; break;
            case 'W': level = ESP_LOG_WARN; break;
            case 'D': level = ESP_LOG_DEBUG; break;
            case 'V': level = ESP_LOG_VERBOSE; break;
            default: break;
            }
            if (length > 0U) jbt_link_log_line(level, text, length);
        }
        inside = false;
    }
    va_end(copy);
    return printed;
}
#endif

static void jbt_link_task(void *arg)
{
    (void)arg;
    uint8_t chunk[256];
    while (true) {
        const int n = uart_read_bytes(JBT_UART, chunk, sizeof(chunk), pdMS_TO_TICKS(100));
        for (int i = 0; i < n; ++i) {
            jbt_frame_t frame;
            if (jbt_decoder_feed(&s_decoder, chunk[i], &frame)) {
                s_handler(&frame, s_context);
            }
        }
    }
}

esp_err_t jbt_link_start(jbt_link_handler_t handler, void *context)
{
    if (handler == NULL) return ESP_ERR_INVALID_ARG;
    if (s_started) return ESP_ERR_INVALID_STATE;
    s_handler = handler;
    s_context = context;
    jbt_decoder_init(&s_decoder);
    s_send_lock = xSemaphoreCreateMutex();
    if (s_send_lock == NULL) return ESP_ERR_NO_MEM;

    const uart_config_t config = {
        .baud_rate = CONFIG_JBT_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(JBT_UART, JBT_RX_RING, JBT_TX_RING, 0, NULL, 0), TAG,
                        "uart driver");
    ESP_RETURN_ON_ERROR(uart_param_config(JBT_UART, &config), TAG, "uart config");
    ESP_RETURN_ON_ERROR(uart_set_pin(JBT_UART, CONFIG_JBT_UART_TX_GPIO, CONFIG_JBT_UART_RX_GPIO,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "uart pins");

    /* Priority above the Bluetooth application tasks and below the
     * controller's: a frame from the host is answered promptly, and nothing
     * here is allowed to starve the radio. */
    if (xTaskCreate(jbt_link_task, "jbt_link", 6144, NULL, 12, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    s_started = true;
#if CONFIG_JBT_LOG_MIRROR
    s_previous_vprintf = esp_log_set_vprintf(jbt_link_vprintf);
#endif
    ESP_LOGI(TAG, "uart%d tx %d rx %d at %d", CONFIG_JBT_UART_NUM, CONFIG_JBT_UART_TX_GPIO,
             CONFIG_JBT_UART_RX_GPIO, CONFIG_JBT_UART_BAUD);
    return ESP_OK;
}

uint32_t jbt_link_dropped(void)
{
    return s_decoder.dropped;
}
