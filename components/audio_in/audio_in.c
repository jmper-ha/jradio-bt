#include "audio_in.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "audio_in";

#define AUDIO_IN_RATE 44100U
/* ~185 ms of 44.1 kHz stereo between the bus and the encoder. The encoder
 * reads in bursts a few tens of milliseconds apart; the DMA delivers 5 ms
 * blocks. Internal RAM, so kept modest. */
#define AUDIO_IN_RING (32U * 1024U)
/* One DMA block read at a time: 256 frames at 16-bit stereo. */
#define AUDIO_IN_BLOCK_FRAMES 256U
#define AUDIO_IN_BLOCK_BYTES (AUDIO_IN_BLOCK_FRAMES * 4U)
/* The bus counts as silent after this long without a full block. */
#define AUDIO_IN_SILENT_US (500 * 1000)

static i2s_chan_handle_t s_channel;
static RingbufHandle_t s_ring;
static TaskHandle_t s_reader;
static SemaphoreHandle_t s_lock;
static bool s_open;
static volatile bool s_run;
static uint32_t s_source_rate = 44100U;
static uint8_t s_source_channels = 2U;
static int64_t s_last_block_us;

/* Linear resampler state, in Q16 fractional frames of the source. */
static uint32_t s_step_q16 = 1U << 16;
static uint32_t s_phase_q16;
static int16_t s_last_left;
static int16_t s_last_right;

static void audio_in_set_ratio(void)
{
    s_step_q16 = (uint32_t)(((uint64_t)s_source_rate << 16) / AUDIO_IN_RATE);
    s_phase_q16 = 0U;
}

/* Turns one block of source frames into 44.1 kHz stereo frames, appending
 * to `out`. `out` must hold the worst case: a 16 kHz source gives almost
 * three frames per input frame. Returns the bytes written. */
static size_t audio_in_resample(const int16_t *in, size_t in_frames, uint8_t channels, int16_t *out)
{
    size_t produced = 0U;
    if (s_step_q16 == (1U << 16) && channels == 2U) {
        memcpy(out, in, in_frames * 4U);
        return in_frames * 4U;
    }
    for (size_t i = 0; i < in_frames; ++i) {
        const int16_t left = channels == 2U ? in[2U * i] : in[i];
        const int16_t right = channels == 2U ? in[2U * i + 1U] : in[i];
        /* Output frames that fall between the previous input frame and this
         * one, at the phase the last one left. */
        while (s_phase_q16 < (1U << 16)) {
            const int32_t f = (int32_t)s_phase_q16;
            out[produced * 2U] = (int16_t)(s_last_left + (((int32_t)left - s_last_left) * f >> 16));
            out[produced * 2U + 1U] = (int16_t)(s_last_right + (((int32_t)right - s_last_right) * f >> 16));
            ++produced;
            s_phase_q16 += s_step_q16;
        }
        s_phase_q16 -= 1U << 16;
        s_last_left = left;
        s_last_right = right;
    }
    return produced * 4U;
}

static void audio_in_reader_task(void *arg)
{
    (void)arg;
    /* Up to three output frames per input frame at the lowest rate. */
    static int16_t in[AUDIO_IN_BLOCK_FRAMES * 2U];
    static int16_t out[AUDIO_IN_BLOCK_FRAMES * 2U * 3U + 8U];
    while (s_run) {
        size_t read = 0U;
        const esp_err_t err = i2s_channel_read(s_channel, in, AUDIO_IN_BLOCK_BYTES, &read, pdMS_TO_TICKS(200));
        if (err != ESP_OK || read == 0U) continue;
        s_last_block_us = esp_timer_get_time();
        const uint8_t channels = s_source_channels;
        /* The channel is opened stereo whatever the host sends; a mono host
         * arrives as pairs of the same sample and is read as such. */
        const size_t frames = read / 4U;
        const size_t bytes = audio_in_resample(in, frames, 2U, out);
        (void)channels;
        if (xRingbufferSend(s_ring, out, bytes, 0) != pdTRUE) {
            /* Full: the encoder is behind. Dropping the block here keeps the
             * latency bounded rather than letting it grow. */
        }
    }
    s_reader = NULL;
    vTaskDelete(NULL);
}

esp_err_t audio_in_open(void)
{
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        if (s_lock == NULL) return ESP_ERR_NO_MEM;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = ESP_OK;
    if (!s_open) {
        if (s_ring == NULL) {
            s_ring = xRingbufferCreate(AUDIO_IN_RING, RINGBUF_TYPE_BYTEBUF);
            if (s_ring == NULL) err = ESP_ERR_NO_MEM;
        }
        if (err == ESP_OK) {
            i2s_chan_config_t channel = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_SLAVE);
            i2s_std_config_t config = {
                .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(s_source_rate),
                /* Philips, 16-bit stereo, the way the host drives the DAC. */
                .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                                I2S_SLOT_MODE_STEREO),
                .gpio_cfg = {
                    .mclk = I2S_GPIO_UNUSED,
                    .bclk = CONFIG_JBT_I2S_BCLK_GPIO,
                    .ws = CONFIG_JBT_I2S_LRCK_GPIO,
                    .dout = I2S_GPIO_UNUSED,
                    .din = CONFIG_JBT_I2S_DATA_GPIO,
                },
            };
            err = i2s_new_channel(&channel, NULL, &s_channel);
            if (err == ESP_OK) err = i2s_channel_init_std_mode(s_channel, &config);
            if (err == ESP_OK) err = i2s_channel_enable(s_channel);
            if (err == ESP_OK) {
                audio_in_set_ratio();
                s_run = true;
                if (xTaskCreate(audio_in_reader_task, "audio_in", 4096, NULL, configMAX_PRIORITIES - 3,
                                &s_reader) != pdPASS) {
                    err = ESP_ERR_NO_MEM;
                    s_run = false;
                }
            }
            if (err == ESP_OK) {
                s_open = true;
                ESP_LOGI(TAG, "listening on bclk %d lrck %d data %d as %u Hz", CONFIG_JBT_I2S_BCLK_GPIO,
                         CONFIG_JBT_I2S_LRCK_GPIO, CONFIG_JBT_I2S_DATA_GPIO, (unsigned)s_source_rate);
            } else if (s_channel != NULL) {
                (void)i2s_channel_disable(s_channel);
                (void)i2s_del_channel(s_channel);
                s_channel = NULL;
            }
        }
    }
    xSemaphoreGive(s_lock);
    return err;
}

void audio_in_close(void)
{
    if (s_lock == NULL) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_open) {
        s_open = false;
        s_run = false;
        /* The reader is inside a 200 ms read at most; wait it out so the
         * channel is not deleted under it. */
        for (int i = 0; i < 30 && s_reader != NULL; ++i) vTaskDelay(pdMS_TO_TICKS(10));
        (void)i2s_channel_disable(s_channel);
        (void)i2s_del_channel(s_channel);
        s_channel = NULL;
        const gpio_num_t pins[] = {CONFIG_JBT_I2S_BCLK_GPIO, CONFIG_JBT_I2S_LRCK_GPIO,
                                   CONFIG_JBT_I2S_DATA_GPIO};
        for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
            (void)gpio_reset_pin(pins[i]);
            (void)gpio_set_direction(pins[i], GPIO_MODE_INPUT);
            (void)gpio_set_pull_mode(pins[i], GPIO_FLOATING);
        }
        size_t length = 0U;
        void *item;
        while ((item = xRingbufferReceiveUpTo(s_ring, &length, 0, AUDIO_IN_RING)) != NULL) {
            vRingbufferReturnItem(s_ring, item);
        }
        ESP_LOGI(TAG, "closed");
    }
    xSemaphoreGive(s_lock);
}

bool audio_in_is_open(void)
{
    return s_open;
}

esp_err_t audio_in_set_format(uint32_t sample_rate, uint8_t bits, uint8_t channels)
{
    if (sample_rate < 8000U || sample_rate > 96000U || bits != 16U || (channels != 1U && channels != 2U)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_lock == NULL) {
        s_source_rate = sample_rate;
        s_source_channels = channels;
        return ESP_OK;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (sample_rate != s_source_rate || channels != s_source_channels) {
        s_source_rate = sample_rate;
        s_source_channels = channels;
        audio_in_set_ratio();
        /* A slave takes its clock from the bus: nothing to reconfigure but
         * the ratio, since the slot stays 16-bit stereo. */
        ESP_LOGI(TAG, "host clocks %u Hz, %u ch -> resample x%.3f", (unsigned)sample_rate, channels,
                 (double)sample_rate / AUDIO_IN_RATE);
    }
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

size_t audio_in_read(uint8_t *out, size_t length)
{
    size_t got = 0U;
    if (s_ring != NULL && s_open) {
        while (got < length) {
            size_t n = 0U;
            uint8_t *item = xRingbufferReceiveUpTo(s_ring, &n, 0, length - got);
            if (item == NULL) break;
            memcpy(&out[got], item, n);
            vRingbufferReturnItem(s_ring, item);
            got += n;
        }
    }
    if (got < length) memset(&out[got], 0, length - got);
    return got;
}

bool audio_in_clocked(void)
{
    return s_open && esp_timer_get_time() - s_last_block_us < AUDIO_IN_SILENT_US;
}
