//
// Created by felix on 10.10.25.
//

#ifndef T2V_MODULE_FW_COMMON_H
#define T2V_MODULE_FW_COMMON_H


#include "FreeRTOS.h"
#include "esp_cpu.h"
#include "esp_log.h"
#include "nvs.h"
#include "queue.h"

#include "t2v_config.h"
#include "t2v_types.h"
#include "t2v_address_config.h"

#ifdef CONFIG_T2V_BLE_ENABLE
#include "ble_task/ble_task.h"
#endif

#ifdef CONFIG_T2V_BTN_INPUT_ENABLE
#include "btn_input_task/btn_input_task.h"
#endif

#ifdef CONFIG_T2V_ETHERNET_ENABLE
#include "ethernet_task/ethernet_task.h"
#endif

#ifdef CONFIG_T2V_IR_NEC_RX_ENABLE
#include "ir_nec_rx_task/ir_nec_rx_task.h"
#endif

#ifdef CONFIG_T2V_IR_NEC_TX_ENABLE
#include "ir_nec_tx_task/ir_nec_tx_task.h"
#endif

#ifdef CONFIG_T2V_LED_DRIVER_ENABLE
#include "led_driver_task/led_driver_task.h"
#endif

#ifdef CONFIG_T2V_USB_DEVICE_ENABLE
#include "usb_device_task/usb_device_task.h"
#endif

#ifdef CONFIG_T2V_WS2812_ENABLE
#include "ws2812_task/ws2812_task.h"
#endif

#ifdef CONFIG_T2V_ZENOH_ENABLE
#include "zenoh_task/zenoh_task.h"
#endif





#endif //T2V_MODULE_FW_COMMON_H
