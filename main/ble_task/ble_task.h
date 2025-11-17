//
// Created by felix on 17.11.25.
//

#ifndef T2V_MOD_FW_BLE_TASK_H
#define T2V_MOD_FW_BLE_TASK_H

#include "FreeRTOS.h"
#include "queue.h"
#include "../t2v_config.h"
#include "../t2v_types.h"

typedef struct BleTaskParams
{
    QueueHandle_t in_ir_nec;
#ifdef CONFIG_T2V_LED_DRIVER_ENABLE
    QueueHandle_t out_led_driver;
#endif
} BleTaskParams_t;

void ble_task_main(void*);

#endif //T2V_MOD_FW_BLE_TASK_H