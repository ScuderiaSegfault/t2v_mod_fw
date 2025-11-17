//
// Created by felix on 17.11.25.
//

#ifndef T2V_MOD_FW_BTN_INPUT_TASK_H
#define T2V_MOD_FW_BTN_INPUT_TASK_H

#include "FreeRTOS.h"
#include "queue.h"

#include "../t2v_config.h"
#include "../t2v_types.h"


typedef struct BtnInputTaskParams
{
    QueueHandleVec_t out_button_updates;
} BtnInputTaskParams_t;

void btn_input_task_main(void*);


#endif //T2V_MOD_FW_BTN_INPUT_TASK_H