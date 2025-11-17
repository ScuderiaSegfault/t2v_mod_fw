//
// Created by felix on 16.11.25.
//

#ifndef T2V_MOD_FW_WS2812_TASK_H
#define T2V_MOD_FW_WS2812_TASK_H

#include "FreeRTOS.h"
#include "../t2v_config.h"

typedef struct Ws2812TaskParams
{

} Ws2812TaskParams_t;
void ws2812_task_main(void*);

#endif //T2V_MOD_FW_WS2812_TASK_H