//
// Created by felix on 11.11.25.
//

#include "ir_nec_tx_task.h"

#include "esp_log.h"
#include "ir_nec_encoder.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_types.h"

void ir_nec_tx_task_main(void * params)
{
#ifdef CONFIG_T2V_CAP_SEND_IR_FRAMES
    IrNecTxTaskParams_t* task_params = params;

    ESP_LOGI(TAG_T2V_MODULE_NEC_TX, "create RMT TX channel");
    rmt_tx_channel_config_t tx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = CONFIG_T2V_RMT_FREQUENCY,
        .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
        .trans_queue_depth = 4,  // number of transactions that allowed to pend in the background, this example won't queue multiple transactions, so queue depth > 1 is sufficient
        .gpio_num = CONFIG_T2V_IR_NEC_TX_GPIO_NUM,
    };
    rmt_channel_handle_t tx_channel = NULL;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_channel_cfg, &tx_channel));

    ESP_LOGI(TAG_T2V_MODULE_NEC_TX, "modulate carrier to TX channel");
    rmt_carrier_config_t carrier_cfg = {
        .duty_cycle = 0.33,
        .frequency_hz = 38000, // 38KHz
    };
    ESP_ERROR_CHECK(rmt_apply_carrier(tx_channel, &carrier_cfg));

    // this example won't send NEC frames in a loop
    rmt_transmit_config_t transmit_config = {
        .loop_count = 0, // no loop
    };

    ESP_LOGI(TAG_T2V_MODULE_NEC_RCV, "install IR NEC encoder");
    const ir_nec_encoder_config_t nec_encoder_cfg = {
        .resolution = CONFIG_T2V_RMT_FREQUENCY,
    };
    rmt_encoder_handle_t nec_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_ir_nec_encoder(&nec_encoder_cfg, &nec_encoder));

    ESP_LOGI(TAG_T2V_MODULE_NEC_RCV, "enable RMT TX and RX channels");
    ESP_ERROR_CHECK(rmt_enable(tx_channel));

    uint8_t frame[4];

    while(1)
    {
        if (xQueueReceive(task_params->in_ir_nec_frames, &frame, pdMS_TO_TICKS(100)))
        {
            const ir_nec_scan_code_t scan_code = {
                .address = (uint16_t) frame[1] << 8 | frame[0],
                .command = (uint16_t) frame[3] << 8 | frame[2],
            };
            ESP_ERROR_CHECK(rmt_transmit(tx_channel, nec_encoder, &scan_code, sizeof(scan_code), &transmit_config));
        }
    }
#else
    ESP_LOGW(TAG_T2V_MODULE_NEC_TX, "No work to do for IR NEC Sender");
    vTaskDelete(NULL);
#endif
}
