//
// Created by felix on 15.11.25.
//

#ifndef T2V_MOD_FW_IR_NEC_TX_TASK_H
#define T2V_MOD_FW_IR_NEC_TX_TASK_H

#include "FreeRTOS.h"
#include "queue.h"
#include "../t2v_config.h"

typedef struct IrNecTxTaskParams
{
#ifdef CONFIG_T2V_CAP_SEND_IR_FRAMES
    QueueHandle_t in_ir_nec_frames;
#endif
} IrNecTxTaskParams_t;

void ir_nec_tx_task_main(void*);

#endif //T2V_MOD_FW_IR_NEC_TX_TASK_H