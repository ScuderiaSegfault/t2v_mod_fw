/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "common.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "portmacro.h"

static TaskHandle_t ir_nec_task = NULL;
static TaskHandle_t usb_device_task = NULL;
static TaskHandle_t ble_device_task = NULL;

static QueueHandle_t queues[2];

static struct QueueSlice queue_slice = {
    .length = 2,
    .capacity = 2,
    .pointer = queues
};

void app_main(void)
{
    ESP_LOGI(TAG_T2V_MODULE, "RoboRacer T2V Module firmware version %d.%d.%d", FW_VERSION_MAJOR, FW_VERSION_MINOR,
             FW_VERSION_PATCH);

    ESP_LOGI(TAG_T2V_MODULE, "Restoring configuration");

    ESP_LOGD(TAG_T2V_MODULE, "Reading from NVS");
    nvs_flash_init();
    nvs_handle_t nvs_handle;
    esp_err_t esp_err = nvs_open(T2V_NVS_STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (esp_err != ESP_OK)
    {
        ESP_LOGE(TAG_T2V_MODULE, "Error (%s) opening NVS handle!", esp_err_to_name(esp_err));
        return;
    }

    uint32_t nsv_address_config;
    esp_err = nvs_get_u32(nvs_handle, T2V_NVS_STORAGE_KEY_ADDRESS_CONFIG, &nsv_address_config);
    if (esp_err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG_T2V_MODULE, "No previous configuration found, initializing configuration.");
        write_address_config(nvs_handle, 0x80);
    }
    else if (esp_err != ESP_OK)
    {
        ESP_LOGE(TAG_T2V_MODULE, "Error (%s) opening NVS handle!", esp_err_to_name(esp_err));
        return;
    }
    else
    {
        ESP_LOGI(TAG_T2V_MODULE, "Loaded configureation from NVS: %08x", nsv_address_config);
        write_address_config_bare(nsv_address_config);
    }

    esp_err = nvs_commit(nvs_handle);
    if (esp_err != ESP_OK)
    {
        ESP_LOGE(TAG_T2V_MODULE, "Error (%s) commiting data to NVS!", esp_err_to_name(esp_err));
        return;
    }

    nvs_close(nvs_handle);

    queues[0] = xQueueCreate(10, sizeof(uint8_t) * 4);
    queues[1] = xQueueCreate(10, sizeof(uint8_t) * 4);

    xTaskCreate(
        ir_nec_task_main,
        "ir_nec_task",
        4096,
        &queue_slice,
        tskIDLE_PRIORITY,
        &ir_nec_task
    );

    xTaskCreate(
        usb_device_task_main,
        "usb_device_task",
        4096,
        queues,
        tskIDLE_PRIORITY,
        &usb_device_task
    );

    xTaskCreate(
        ble_task_main,
        "ble_task",
        4096,
        queues + 1,
        tskIDLE_PRIORITY,
        &ble_device_task
    );
}
