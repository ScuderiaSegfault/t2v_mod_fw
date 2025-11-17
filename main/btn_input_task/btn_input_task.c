//
// Created by felix on 11.11.25.
//

#include "driver/gpio.h"
#include "btn_input_task.h"

#include "esp_log.h"

#define ESP_INTR_FLAG_DEFAULT 0

static QueueHandle_t gpio_evt_queue;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    const uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void btn_input_task_main(void* params)
{
    const BtnInputTaskParams_t* task_params = params;

    ESP_LOGD(TAG_T2V_MODULE_BTN_INPUT, "starting btn_input task");
    gpio_config_t btn_config;

    btn_config.intr_type = GPIO_INTR_ANYEDGE;
    btn_config.mode = GPIO_MODE_INPUT;
    btn_config.pin_bit_mask = (1ULL << CONFIG_T2V_BTN_INPUT_SW1_GPIO_NUM) | (1ULL << CONFIG_T2V_BTN_INPUT_SW2_GPIO_NUM);
    btn_config.pull_down_en = 0;
    btn_config.pull_up_en = 1;

    gpio_config(&btn_config);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(CONFIG_T2V_BTN_INPUT_SW1_GPIO_NUM, gpio_isr_handler, (void*) CONFIG_T2V_BTN_INPUT_SW1_GPIO_NUM);
    gpio_isr_handler_add(CONFIG_T2V_BTN_INPUT_SW2_GPIO_NUM, gpio_isr_handler, (void*) CONFIG_T2V_BTN_INPUT_SW2_GPIO_NUM);

    ESP_LOGD(TAG_T2V_MODULE_BTN_INPUT, "GPIOs configured");

    uint32_t io_num;

    uint32_t btn_0_level = gpio_get_level(CONFIG_T2V_BTN_INPUT_SW1_GPIO_NUM);
    uint32_t btn_1_level = gpio_get_level(CONFIG_T2V_BTN_INPUT_SW2_GPIO_NUM);

    while(1)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            const uint32_t btn_level = gpio_get_level(io_num);
            ESP_LOGD(TAG_T2V_MODULE_BTN_INPUT, "GPIO[%"PRIu32"] intr, val: %d", io_num, btn_level);
            ButtonUpdate_t update = { 0 };
            if(io_num == CONFIG_T2V_BTN_INPUT_SW1_GPIO_NUM)
            {
                if (btn_0_level == 0 && btn_level == 1)
                {
                    update.btn0_event = BUTTON_EVENT_RELEASED;
                    btn_0_level = 1;
                } else if (btn_0_level == 1 && btn_level == 0)
                {
                    update.btn0_event = BUTTON_EVENT_PRESSED;
                    btn_0_level = 0;
                } else
                {
                    continue;
                }
            }
            if(io_num == CONFIG_T2V_BTN_INPUT_SW2_GPIO_NUM)
            {
                if (btn_1_level == 0 && btn_level == 1)
                {
                    update.btn1_event = BUTTON_EVENT_RELEASED;
                    btn_1_level = 1;
                } else if (btn_1_level == 1 && btn_level == 0)
                {
                    update.btn1_event = BUTTON_EVENT_PRESSED;
                    btn_1_level = 0;
                } else
                {
                    continue;
                }
            }

            for (size_t i = 0; i < task_params->out_button_updates.len; i++)
            {
                xQueueSendToBack(task_params->out_button_updates.pointer[i], &update, pdMS_TO_TICKS(100));
            }
        }
    }
}
