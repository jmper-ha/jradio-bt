#include "audio_out.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "audio_out";

/* About 90 ms of 44.1 kHz stereo. Enough to ride out the Bluetooth stack's
 * scheduling jitter; small enough that a track change is not heard late.
 * Internal RAM: there is no PSRAM on this module. */
#define AUDIO_RING_SIZE (16U * 1024U)
#define AUDIO_PREFETCH (AUDIO_RING_SIZE / 4U)
/* One DMA descriptor's worth per write, so the writer never holds more than
 * it can hand over at once. */
#define AUDIO_CHUNK 960U

static i2s_chan_handle_t s_channel;
static RingbufHandle_t s_ring;
static TaskHandle_t s_writer;
static SemaphoreHandle_t s_lock;
static uint32_t s_sample_rate = 44100U;
static uint8_t s_channels = 2U;
static bool s_claimed;
static bool s_streaming;
static bool s_prefetching = true;
/* Q15 gain from the 0..127 volume: the square of the fraction, which is
 * roughly how a phone expects its slider to feel - linear would sit loud
 * over most of its travel. */
static volatile int32_t s_gain_q15 = 32767;

static void audio_out_apply_gain(int16_t *samples, size_t count)
{
    const int32_t gain = s_gain_q15;
    if (gain >= 32767) return;
    for (size_t i = 0; i < count; ++i) {
        samples[i] = (int16_t)(((int32_t)samples[i] * gain) >> 15);
    }
}

static void audio_out_writer_task(void *arg)
{
    (void)arg;
    while (true) {
        if (s_prefetching) {
            /* Wait for the buffer to fill some, then drain steadily. The
             * wait is short so a suspend/resume is not a long silence. */
            if (s_ring == NULL || xRingbufferGetCurFreeSize(s_ring) > AUDIO_RING_SIZE - AUDIO_PREFETCH) {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            s_prefetching = false;
        }
        size_t length = 0U;
        uint8_t *chunk = xRingbufferReceiveUpTo(s_ring, &length, pdMS_TO_TICKS(50), AUDIO_CHUNK);
        if (chunk == NULL) {
            /* Ran dry: back to filling. Expected on pause, a defect while
             * playing - counted by the stack's own stats, not here. */
            s_prefetching = true;
            continue;
        }
        audio_out_apply_gain((int16_t *)chunk, length / 2U);
        if (s_claimed && s_streaming) {
            size_t written = 0U;
            (void)i2s_channel_write(s_channel, chunk, length, &written, portMAX_DELAY);
        }
        vRingbufferReturnItem(s_ring, chunk);
    }
}

static esp_err_t audio_out_configure(void)
{
    const i2s_std_clk_config_t clock = I2S_STD_CLK_DEFAULT_CONFIG(s_sample_rate);
    /* Philips timing, as the PCM5102 on the jRadio board is wired for and
     * as the S3 drives it. */
    const i2s_std_slot_config_t slot = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
        I2S_DATA_BIT_WIDTH_16BIT, s_channels == 1U ? I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO);
    ESP_RETURN_ON_ERROR(i2s_channel_reconfig_std_clock(s_channel, &clock), TAG, "clock");
    ESP_RETURN_ON_ERROR(i2s_channel_reconfig_std_slot(s_channel, &slot), TAG, "slot");
    return ESP_OK;
}

esp_err_t audio_out_claim(void)
{
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        if (s_lock == NULL) return ESP_ERR_NO_MEM;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = ESP_OK;
    if (!s_claimed) {
        if (s_ring == NULL) {
            s_ring = xRingbufferCreate(AUDIO_RING_SIZE, RINGBUF_TYPE_BYTEBUF);
            if (s_ring == NULL) err = ESP_ERR_NO_MEM;
        }
        if (err == ESP_OK && s_writer == NULL &&
            xTaskCreate(audio_out_writer_task, "audio_out", 3072, NULL, configMAX_PRIORITIES - 3,
                        &s_writer) != pdPASS) {
            err = ESP_ERR_NO_MEM;
        }
        if (err == ESP_OK) {
            i2s_chan_config_t channel = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
            /* Silence, not the last buffer, when the writer is late. */
            channel.auto_clear = true;
            i2s_std_config_t config = {
                .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(s_sample_rate),
                .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                                I2S_SLOT_MODE_STEREO),
                .gpio_cfg = {
                    .mclk = I2S_GPIO_UNUSED,
                    .bclk = CONFIG_JBT_I2S_BCLK_GPIO,
                    .ws = CONFIG_JBT_I2S_LRCK_GPIO,
                    .dout = CONFIG_JBT_I2S_DATA_GPIO,
                    .din = I2S_GPIO_UNUSED,
                },
            };
            err = i2s_new_channel(&channel, &s_channel, NULL);
            if (err == ESP_OK) err = i2s_channel_init_std_mode(s_channel, &config);
            if (err == ESP_OK) err = audio_out_configure();
            if (err == ESP_OK) err = i2s_channel_enable(s_channel);
            if (err == ESP_OK) {
                s_claimed = true;
                s_prefetching = true;
                ESP_LOGI(TAG, "bus claimed: bclk %d lrck %d data %d, %u Hz", CONFIG_JBT_I2S_BCLK_GPIO,
                         CONFIG_JBT_I2S_LRCK_GPIO, CONFIG_JBT_I2S_DATA_GPIO, (unsigned)s_sample_rate);
            } else if (s_channel != NULL) {
                (void)i2s_del_channel(s_channel);
                s_channel = NULL;
            }
        }
    }
    xSemaphoreGive(s_lock);
    return err;
}

void audio_out_release(void)
{
    if (s_lock == NULL) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_claimed) {
        s_claimed = false;
        s_streaming = false;
        (void)i2s_channel_disable(s_channel);
        (void)i2s_del_channel(s_channel);
        s_channel = NULL;
        /* Back to plain inputs, no pull: the host drives these next, and a
         * pull-up on our side would fight its edges. */
        const gpio_num_t pins[] = {CONFIG_JBT_I2S_BCLK_GPIO, CONFIG_JBT_I2S_LRCK_GPIO,
                                   CONFIG_JBT_I2S_DATA_GPIO};
        for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
            (void)gpio_reset_pin(pins[i]);
            (void)gpio_set_direction(pins[i], GPIO_MODE_INPUT);
            (void)gpio_set_pull_mode(pins[i], GPIO_FLOATING);
        }
        ESP_LOGI(TAG, "bus released");
    }
    xSemaphoreGive(s_lock);
}

bool audio_out_is_claimed(void)
{
    return s_claimed;
}

esp_err_t audio_out_set_format(uint32_t sample_rate, uint8_t channels)
{
    if (sample_rate == 0U || (channels != 1U && channels != 2U)) return ESP_ERR_INVALID_ARG;
    if (s_lock == NULL) {
        s_sample_rate = sample_rate;
        s_channels = channels;
        return ESP_OK;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = ESP_OK;
    if (sample_rate != s_sample_rate || channels != s_channels) {
        s_sample_rate = sample_rate;
        s_channels = channels;
        if (s_claimed) {
            /* The clock can only change on a disabled channel. */
            const bool was_streaming = s_streaming;
            s_streaming = false;
            (void)i2s_channel_disable(s_channel);
            err = audio_out_configure();
            (void)i2s_channel_enable(s_channel);
            s_streaming = was_streaming;
            ESP_LOGI(TAG, "format %u Hz, %u ch", (unsigned)sample_rate, channels);
        }
    }
    xSemaphoreGive(s_lock);
    return err;
}

void audio_out_stream(bool started)
{
    s_streaming = started;
    if (!started && s_ring != NULL) {
        /* Drain what is queued so the next start does not replay it. */
        size_t length = 0U;
        void *item;
        while ((item = xRingbufferReceiveUpTo(s_ring, &length, 0, AUDIO_RING_SIZE)) != NULL) {
            vRingbufferReturnItem(s_ring, item);
        }
        s_prefetching = true;
    }
}

size_t audio_out_write(const uint8_t *pcm, size_t length)
{
    if (!s_claimed || !s_streaming || s_ring == NULL) return length; /* nowhere to go: dropped */
    if (xRingbufferSend(s_ring, pcm, length, 0) != pdTRUE) {
        return 0U;
    }
    return length;
}

void audio_out_set_volume(uint8_t volume)
{
    if (volume > 127U) volume = 127U;
    s_gain_q15 = (int32_t)volume * (int32_t)volume * 32767 / (127 * 127);
}
