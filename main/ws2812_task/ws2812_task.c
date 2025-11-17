/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <string.h>

#include "ws2812_task.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"
#include "driver/gpio.h"

static uint8_t led_strip_pixels[CONFIG_T2V_WS2812_LED_COUNT * 3];

void ws2812_task_main(void* params)
{
    Ws2812TaskParams_t* task_params = params;
    (void) task_params;

    static uint8_t changing_colors[CONFIG_T2V_WS2812_LED_COUNT * 3];

    ESP_LOGI(TAG_T2V_MODULE_WS2812, "Create RMT TX channel");
    rmt_channel_handle_t led_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = CONFIG_T2V_WS2812_GPIO_NUM,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .resolution_hz = CONFIG_T2V_WS2812_RMT_FREQUENCY,
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
        .flags.invert_out = true,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    ESP_LOGI(TAG_T2V_MODULE_WS2812, "Install led strip encoder");
    rmt_encoder_handle_t led_encoder = NULL;
    led_strip_encoder_config_t encoder_config = {
        .resolution = CONFIG_T2V_WS2812_RMT_FREQUENCY,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    ESP_LOGI(TAG_T2V_MODULE_WS2812, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_chan));

    ESP_LOGI(TAG_T2V_MODULE_WS2812, "Start LED rainbow chase");
    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };

    vTaskDelay(pdMS_TO_TICKS(1000));

    for (size_t i = 0; i < CONFIG_T2V_WS2812_LED_COUNT; i++)
    {
        changing_colors[i * 3 + 0] = 0x7f;
        changing_colors[i * 3 + 1] = 0x00;
        changing_colors[i * 3 + 2] = 0x00;
    }

    ESP_LOGD(TAG_T2V_MODULE_WS2812, "Sending initial value");
    ESP_LOG_BUFFER_HEX(TAG_T2V_MODULE_WS2812, led_strip_pixels, sizeof(uint8_t) * CONFIG_T2V_WS2812_LED_COUNT * 3);

    // Flush RGB values to LEDs
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, 24, &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));

    ESP_LOGD(TAG_T2V_MODULE_WS2812, "Finished sending initial value");

    vTaskDelay(pdMS_TO_TICKS(250));

    size_t inverted_led = 0;

    while (1) {
        memcpy(led_strip_pixels, changing_colors, sizeof(uint8_t) * CONFIG_T2V_WS2812_LED_COUNT * 3);

        led_strip_pixels[inverted_led * 3 + 0] = 0x7f - led_strip_pixels[inverted_led * 3 + 0];
        led_strip_pixels[inverted_led * 3 + 1] = 0x7f - led_strip_pixels[inverted_led * 3 + 1];
        led_strip_pixels[inverted_led * 3 + 2] = 0x7f - led_strip_pixels[inverted_led * 3 + 2];

        inverted_led = (inverted_led + 1) % CONFIG_T2V_WS2812_LED_COUNT;

        // Flush RGB values to LEDs
        ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, 24, &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}