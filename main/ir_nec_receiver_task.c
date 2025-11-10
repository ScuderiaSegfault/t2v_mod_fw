/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */


#include "common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "ir_nec_encoder.h"


/**
 * @brief NEC timing spec
 */
#define NEC_LEADING_CODE_DURATION_0  9000
#define NEC_LEADING_CODE_DURATION_1  4500
#define NEC_PAYLOAD_ZERO_DURATION_0  560
#define NEC_PAYLOAD_ZERO_DURATION_1  560
#define NEC_PAYLOAD_ONE_DURATION_0   560
#define NEC_PAYLOAD_ONE_DURATION_1   1690
#define NEC_REPEAT_CODE_DURATION_0   9000
#define NEC_REPEAT_CODE_DURATION_1   2250

static struct QueueSlice* ir_data_queue;

uint32_t address_config = 0x000110;

/**
 * @brief Saving NEC decode results
 */
static uint16_t s_nec_code_address;
static uint16_t s_nec_code_command;

/**
 * @brief Check whether a duration is within expected range
 */
static bool nec_check_in_range(const uint32_t signal_duration, const uint32_t spec_duration)
{
    return (signal_duration < (spec_duration + IR_NEC_DECODE_MARGIN)) &&
        (signal_duration > (spec_duration - IR_NEC_DECODE_MARGIN));
}

/**
 * @brief Check whether a RMT symbol represents NEC logic zero
 */
static bool nec_parse_logic0(const rmt_symbol_word_t* rmt_nec_symbols)
{
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_PAYLOAD_ZERO_DURATION_0) &&
        nec_check_in_range(rmt_nec_symbols->duration1, NEC_PAYLOAD_ZERO_DURATION_1);
}

/**
 * @brief Check whether a RMT symbol represents NEC logic one
 */
static bool nec_parse_logic1(const rmt_symbol_word_t* rmt_nec_symbols)
{
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_PAYLOAD_ONE_DURATION_0) &&
        nec_check_in_range(rmt_nec_symbols->duration1, NEC_PAYLOAD_ONE_DURATION_1);
}

/**
 * @brief Decode RMT symbols into NEC address and command
 */
static bool nec_parse_frame(rmt_symbol_word_t* rmt_nec_symbols)
{
    const rmt_symbol_word_t* cur = rmt_nec_symbols;
    uint16_t address = 0;
    uint16_t command = 0;
    const bool valid_leading_code = nec_check_in_range(cur->duration0, NEC_LEADING_CODE_DURATION_0) &&
        nec_check_in_range(cur->duration1, NEC_LEADING_CODE_DURATION_1);
    if (!valid_leading_code)
    {
        return false;
    }
    cur++;
    for (int i = 0; i < 16; i++)
    {
        if (nec_parse_logic1(cur))
        {
            address |= 1 << i;
        }
        else if (nec_parse_logic0(cur))
        {
            address &= ~(1 << i);
        }
        else
        {
            return false;
        }
        cur++;
    }
    for (int i = 0; i < 16; i++)
    {
        if (nec_parse_logic1(cur))
        {
            command |= 1 << i;
        }
        else if (nec_parse_logic0(cur))
        {
            command &= ~(1 << i);
        }
        else
        {
            return false;
        }
        cur++;
    }
    // save address and command
    s_nec_code_address = address;
    s_nec_code_command = command;
    return true;
}

/**
 * @brief Check whether the RMT symbols represent NEC repeat code
 */
static bool nec_parse_frame_repeat(const rmt_symbol_word_t* rmt_nec_symbols)
{
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_REPEAT_CODE_DURATION_0) &&
        nec_check_in_range(rmt_nec_symbols->duration1, NEC_REPEAT_CODE_DURATION_1);
}

/**
 * @brief Decode RMT symbols into NEC scan code and send the data to other tasks
 */
static void parse_and_send_nec_frame(rmt_symbol_word_t* rmt_nec_symbols, size_t symbol_num)
{
    ESP_LOGD(TAG_T2V_MODULE_NEC_RCV, "NEC frame start---");
    for (size_t i = 0; i < symbol_num; i++)
    {
        ESP_LOGD(TAG_T2V_MODULE_NEC_RCV, "{%d:%d},{%d:%d}", rmt_nec_symbols[i].level0, rmt_nec_symbols[i].duration0,
                 rmt_nec_symbols[i].level1, rmt_nec_symbols[i].duration1);
    }
    ESP_LOGD(TAG_T2V_MODULE_NEC_RCV, "---NEC frame end: ");
    // decode RMT symbols
    switch (symbol_num)
    {
    case 34: // NEC normal frame
        if (nec_parse_frame(rmt_nec_symbols))
        {
            ESP_LOGI(TAG_T2V_MODULE_NEC_RCV, "Address=%04X, Command=%04X", s_nec_code_address, s_nec_code_command);

            const uint8_t frame_address = *(unsigned char*)&s_nec_code_address;
            if (frame_address >= 0xf0 && frame_address <= 0xfe)
            {
                // reserved addresses, ignore
                ESP_LOGW(TAG_T2V_MODULE_NEC_RCV, "Received frame with reserved address %02x", frame_address);
                break;
            }
            if (frame_address <= 0x0f) // multicast address, check multicast mask
            {
                const uint16_t multicast_mask = read_multicast_mask();
                const uint16_t address_bit = 0x01 << frame_address;

                if ((multicast_mask & address_bit) != address_bit)
                {
                    ESP_LOGD(TAG_T2V_MODULE_NEC_RCV, "Received frame with unregistered multicast address %02x",
                             frame_address);
                    break;
                }
            }
            else if (frame_address != 0xff)
            // only unicast addresses and broadcast remaining, broadcast is always forwarded
            {
                const uint8_t unicast_address = read_unicast_address();
                if (unicast_address != frame_address)
                {
                    ESP_LOGD(TAG_T2V_MODULE_NEC_RCV, "Received frame for different unicast address %02x",
                             frame_address);
                    break;
                }
            }

            unsigned char data[4];
            data[1] = *((unsigned char*)&s_nec_code_address + 1);
            data[0] = *(unsigned char*)&s_nec_code_address;
            data[3] = *((unsigned char*)&s_nec_code_command + 1);
            data[2] = *(unsigned char*)&s_nec_code_command;

            for (size_t i = 0; i < ir_data_queue->length; i++)
            {
                if (xQueueSendToBack(ir_data_queue->pointer[i], data, 0))
                {
                    ESP_LOGD(TAG_T2V_MODULE_NEC_RCV, "NEC IR data sent");
                }
                else
                {
                    ESP_LOGE(TAG_T2V_MODULE_NEC_RCV, "NEC IR data sent failed");
                }
            }
        }
        else
        {
            ESP_LOGW(TAG_T2V_MODULE_NEC_RCV, "Unable to parse NEC frame");
        }
        break;
    case 2: // NEC repeat frame
        if (nec_parse_frame_repeat(rmt_nec_symbols))
        {
            ESP_LOGD(TAG_T2V_MODULE_NEC_RCV, "Address=%04X, Command=%04X, repeat", s_nec_code_address,
                     s_nec_code_command);
        }
        break;
    default:
        ESP_LOGW(TAG_T2V_MODULE_NEC_RCV, "Unknown NEC frame");
        break;
    }
}

static bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t* edata, void* user_data)
{
    (void)channel;

    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = user_data;
    // send the received RMT symbols to the parser task
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

void ir_nec_task_main(void* data)
{
    ir_data_queue = (struct QueueSlice*)data;

    ESP_LOGI(TAG_T2V_MODULE_NEC_RCV, "create RMT RX channel");
    const rmt_rx_channel_config_t rx_channel_cfg = {
        .gpio_num = IR_RX_GPIO_NUM,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = IR_RESOLUTION_HZ,
        .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
    };
    rmt_channel_handle_t rx_channel = NULL;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_cfg, &rx_channel));

    ESP_LOGI(TAG_T2V_MODULE_NEC_RCV, "register RX done callback");
    QueueHandle_t receive_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    assert(receive_queue);
    const rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, receive_queue));

    // the following timing requirement is based on NEC protocol
    const rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 1250,
        // the shortest duration for NEC signal is 560us, 1250ns < 560us, valid signal won't be treated as noise
        .signal_range_max_ns = 12000000,
        // the longest duration for NEC signal is 9000us, 12000000ns > 9000us, the receive won't stop early
    };

    ESP_LOGI(TAG_T2V_MODULE_NEC_RCV, "install IR NEC encoder");
    const ir_nec_encoder_config_t nec_encoder_cfg = {
        .resolution = IR_RESOLUTION_HZ,
    };
    rmt_encoder_handle_t nec_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_ir_nec_encoder(&nec_encoder_cfg, &nec_encoder));

    ESP_LOGI(TAG_T2V_MODULE_NEC_RCV, "enable RMT RX channel");
    ESP_ERROR_CHECK(rmt_enable(rx_channel));

    // save the received RMT symbols
    rmt_symbol_word_t raw_symbols[64]; // 64 symbols should be sufficient for a standard NEC frame
    rmt_rx_done_event_data_t rx_data;
    // ready to receive
    ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
    while (true)
    {
        // wait for RX done signal
        if (xQueueReceive(receive_queue, &rx_data, pdMS_TO_TICKS(1000)) == pdPASS)
        {
            // parse the received symbols and print the result
            parse_and_send_nec_frame(rx_data.received_symbols, rx_data.num_symbols);
            // start receive again
            ESP_ERROR_CHECK(rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &receive_config));
        }
    }
}
