/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "common.h"
#include "esp_log.h"
#include "portmacro.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static TaskHandle_t ir_nec_task = NULL;
static TaskHandle_t usb_device_task = NULL;


static QueueHandle_t ir_nec_queue;

void app_main(void) {
    ESP_LOGI(TAG_T2V_MODULE, "RoboRacer T2V Module firmware version %d.%d.%d", FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH);

    ir_nec_queue = xQueueCreate(10, sizeof(uint8_t) * 4);

    xTaskCreate(
        ir_nec_task_main,
        "ir_nec_task",
        4096,
        &ir_nec_queue,
        tskIDLE_PRIORITY,
        &ir_nec_task
    );

    xTaskCreate(
        usb_device_task_main,
        "usb_device_task",
        4096,
        &ir_nec_queue,
        tskIDLE_PRIORITY,
        &usb_device_task
    );
}
