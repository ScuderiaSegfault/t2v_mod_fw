//
// Created by felix on 15.11.25.
//

#ifndef T2V_MOD_FW_CONFIG_H
#define T2V_MOD_FW_CONFIG_H

#include "FreeRTOSConfig.h"

// Checks for multiple assignments to GPIOs

#ifdef CONFIG_T2V_USE_PCB
#ifdef CONFIG_T2V_IR_NEC_TX_GPIO_07
#ifdef CONFIG_T2V_WS2812_ENABLE
#error "Use of IR NEC (GPIO7) conflicts with use of WS2812"
#endif
#endif
#endif


#define FW_VERSION_MAJOR (0)
#define FW_VERSION_MINOR (1)
#define FW_VERSION_PATCH (0)

#define TAG_T2V_MODULE "tv2_module"
#define TAG_T2V_MODULE_NEC_RCV "t2v_module::nec::rx"
#define TAG_T2V_MODULE_NEC_DECODER "t2v_module::nec::decoder"
#define TAG_T2V_MODULE_USB "t2v_module::usb"
#define TAG_T2V_MODULE_BLE "t2v_module::ble"
#define TAG_T2V_MODULE_LED "t2v_module::led"
#ifdef CONFIG_T2V_IR_NEC_TX_ENABLE
#define TAG_T2V_MODULE_NEC_TX "t2v_module::nec::tx"
#endif
#define TAG_T2V_MODULE_BTN_INPUT "t2v_module::btn::input"
#define TAG_T2V_MODULE_WS2812 "t2v_module::ws2812"
#define TAG_T2V_MODULE_ETH "t2v_module::eth"
#ifdef CONFIG_T2V_ZENOH_ENABLE
#define TAG_T2V_MODULE_ZENOH "t2v_module::zenoh"
#endif


#define RMT_RESOLUTION_HZ     CONFIG_T2V_RMT_FREQUENCY

#endif //T2V_MOD_FW_CONFIG_H