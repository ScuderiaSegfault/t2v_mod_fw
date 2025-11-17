//
// Created by felix on 17.11.25.
//

#ifndef T2V_MOD_FW_LED_DRIVER_TASK_H
#define T2V_MOD_FW_LED_DRIVER_TASK_H

#include "FreeRTOS.h"
#include "queue.h"

#include "../t2v_config.h"
#include "../t2v_types.h"

typedef struct LedDriverParams
{
    QueueHandle_t in_led_updates;
} LedDriverParams_t;

void led_driver_task_main(void*);

#endif //T2V_MOD_FW_LED_DRIVER_TASK_H