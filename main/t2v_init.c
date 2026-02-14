/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "t2v.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "portmacro.h"


#ifdef CONFIG_T2V_BLE_ENABLE
static BleTaskParams_t ble_task_params;
static TaskHandle_t ble_device_task = NULL;
#endif

#ifdef CONFIG_T2V_BTN_INPUT_ENABLE
static BtnInputTaskParams_t btn_input_task_params;
static TaskHandle_t btn_input_task = NULL;
#endif

#ifdef CONFIG_T2V_ETHERNET_ENABLE
static EthernetTaskParams_t eth_task_params;
static TaskHandle_t ethernet_task = NULL;
#endif

#ifdef CONFIG_T2V_IR_NEC_RX_ENABLE
static IrNecRxTaskParams_t ir_nec_rx_task_params;
static TaskHandle_t ir_nec_rx_task = NULL;
#endif

#ifdef CONFIG_T2V_IR_NEC_TX_ENABLE
static IrNecTxTaskParams_t ir_nec_tx_task_params;
static TaskHandle_t ir_nec_tx_task = NULL;
#endif

#ifdef CONFIG_T2V_LED_DRIVER_ENABLE
static LedDriverParams_t led_driver_params;
static TaskHandle_t led_driver_task = NULL;
#endif

#ifdef CONFIG_T2V_USB_DEVICE_ENABLE
static UsbDeviceTaskParams_t usb_device_task_params;
static TaskHandle_t usb_device_task = NULL;
#endif

#ifdef CONFIG_T2V_WS2812_ENABLE
static Ws2812TaskParams_t ws2812_task_params;
static TaskHandle_t ws2812_task = NULL;
#endif

#ifdef CONFIG_T2V_ZENOH_ENABLE
static ZenohTaskParams_t zenoh_task_params;
static TaskHandle_t zenoh_task = NULL;
#endif

void app_main(void)
{
    ESP_LOGI(TAG_T2V_MODULE, "RoboRacer T2V Module firmware version %d.%d.%d", FW_VERSION_MAJOR, FW_VERSION_MINOR,
             FW_VERSION_PATCH);

#ifdef CONFIG_T2V_ADDRESS_CONFIG_ENABLE
    ESP_LOGI(TAG_T2V_MODULE, "Restoring address configuration");

    ESP_LOGD(TAG_T2V_MODULE, "Reading from NVS");
    nvs_flash_init();
    nvs_handle_t nvs_handle;
    esp_err_t esp_err = nvs_open(CONFIG_T2V_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (esp_err != ESP_OK)
    {
        ESP_LOGE(TAG_T2V_MODULE, "Error (%s) opening NVS handle!", esp_err_to_name(esp_err));
        return;
    }

    uint32_t nsv_address_config;
    esp_err = nvs_get_u32(nvs_handle, CONFIG_T2V_ADDRESS_CONFIG_NVS_KEY, &nsv_address_config);
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
        ESP_LOGI(TAG_T2V_MODULE, "Loaded configuration from NVS: %08x", nsv_address_config);
        write_address_config_bare(nsv_address_config);
    }

    esp_err = nvs_commit(nvs_handle);
    if (esp_err != ESP_OK)
    {
        ESP_LOGE(TAG_T2V_MODULE, "Error (%s) commiting data to NVS!", esp_err_to_name(esp_err));
        return;
    }

    nvs_close(nvs_handle);
#endif

#ifdef CONFIG_T2V_IR_NEC_RX_ENABLE
    queue_handle_vec_with_capacity(&ir_nec_rx_task_params.out_ir_nec_frames, IrNecRxQueueLast);
    queue_handle_vec_push(&ir_nec_rx_task_params.out_ir_nec_frames, xQueueCreate(10, sizeof(uint8_t) * 4));
#ifdef CONFIG_T2V_BLE_ENABLE
    queue_handle_vec_push(&ir_nec_rx_task_params.out_ir_nec_frames, xQueueCreate(10, sizeof(uint8_t) * 4));
#endif
    ESP_LOGI(TAG_T2V_MODULE, "Initialized %d IR NEC RX queue(s)", IrNecRxQueueLast);
#endif

#ifdef CONFIG_T2V_USB_DEVICE_ENABLE
    usb_device_task_params.in_ir_data = ir_nec_rx_task_params.out_ir_nec_frames.pointer[IrNecRxQueueUsbTask];
#endif


#ifdef CONFIG_T2V_LED_DRIVER_ENABLE
    led_driver_params.in_led_updates = xQueueCreate(10, sizeof(LedUpdate_t));
#endif

#ifdef CONFIG_T2V_BLE_ENABLE
    ble_task_params.in_ir_nec = ir_nec_rx_task_params.out_ir_nec_frames.pointer[IrNecRxQueueBleTask];
#ifdef CONFIG_T2V_LED_DRIVER_ENABLE
    ble_task_params.out_led_driver = led_driver_params.in_led_updates;
#endif
#endif

#ifdef CONFIG_T2V_IR_NEC_TX_ENABLE
#ifdef CONFIG_T2V_CAP_SEND_IR_FRAMES
    ir_nec_tx_task_params.in_ir_nec_frames = xQueueCreate(10, sizeof(uint8_t) * 4);
#endif
#endif

#ifdef CONFIG_T2V_ZENOH_ENABLE
    zenoh_task_params.in_nic_status = xQueueCreate(10, sizeof(NicStatus_t));
#ifdef CONFIG_T2V_LED_DRIVER_ENABLE
    zenoh_task_params.out_led_driver = led_driver_params.in_led_updates;
#endif
#ifdef CONFIG_T2V_CAP_SEND_IR_FRAMES
    zenoh_task_params.out_ir_frame = ir_nec_tx_task_params.in_ir_nec_frames;
#endif
#endif

#ifdef CONFIG_T2V_ETHERNET_ENABLE
    queue_handle_vec_with_capacity(&eth_task_params.out_nic_status, EthernetTaskQueueLast);
#ifdef CONFIG_T2V_ZENOH_ENABLE
    queue_handle_vec_push(&eth_task_params.out_nic_status, zenoh_task_params.in_nic_status);
#endif
#endif

#ifdef CONFIG_T2V_IR_NEC_RX_ENABLE
    xTaskCreate(
        ir_nec_task_main,
        "ir_nec_task",
        4096,
        &ir_nec_rx_task_params,
        tskIDLE_PRIORITY,
        &ir_nec_rx_task
    );
#endif

#ifdef CONFIG_T2V_USB_DEVICE_ENABLE
    xTaskCreate(
        usb_device_task_main,
        "usb_device_task",
        4096,
        &usb_device_task_params,
        tskIDLE_PRIORITY,
        &usb_device_task
    );
#endif

#ifdef CONFIG_T2V_BLE_ENABLE
    xTaskCreate(
        ble_task_main,
        "ble_task",
        4096,
        &ble_task_params,
        tskIDLE_PRIORITY,
        &ble_device_task
    );
#endif

#ifdef CONFIG_T2V_LED_DRIVER_ENABLE
    xTaskCreate(
        led_driver_task_main,
        "led_driver_task",
        4096,
        &led_driver_params,
        tskIDLE_PRIORITY,
        &led_driver_task
    );
#endif

#ifdef CONFIG_T2V_IR_NEC_TX_ENABLE
    xTaskCreate(
        ir_nec_tx_task_main,
        "ir_nec_tx_task",
        4096,
        &ir_nec_tx_task_params,
        tskIDLE_PRIORITY,
        &ir_nec_tx_task
    );
#endif

#ifdef CONFIG_T2V_BTN_INPUT_ENABLE
    xTaskCreate(
        btn_input_task_main,
        "btn_input_task",
        2048,
        &btn_input_task_params,
        tskIDLE_PRIORITY,
        &btn_input_task
    );
#endif

#ifdef CONFIG_T2V_WS2812_ENABLE
    xTaskCreate(
        ws2812_task_main,
        "ws2812_task",
        4096,
        &ws2812_task_params,
        tskIDLE_PRIORITY,
        &ws2812_task
    );
#endif

#ifdef CONFIG_T2V_ETHERNET_ENABLE
    xTaskCreate(
        ethernet_task_main,
        "ethernet_task",
        4096,
        &eth_task_params,
        tskIDLE_PRIORITY,
        &ethernet_task
    );
#endif

#ifdef CONFIG_T2V_ZENOH_ENABLE
    xTaskCreate(
        zenoh_task_main,
        "zenoh_task",
        4096,
        &zenoh_task_params,
        tskIDLE_PRIORITY,
        &zenoh_task
    );
#endif
}
