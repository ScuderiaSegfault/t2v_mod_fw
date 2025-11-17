//
// Created by felix on 17.11.25.
//

#ifndef T2V_MOD_FW_IR_NEC_RX_TASK_H
#define T2V_MOD_FW_IR_NEC_RX_TASK_H

#include "FreeRTOS.h"
#include "queue.h"

#include "../t2v_config.h"
#include "../t2v_types.h"

enum IrNecRxQueueIds
{
    IrNecRxQueueUsbTask,
#ifdef CONFIG_T2V_BLE_ENABLE
    IrNecRxQueueBleTask,
#endif
    IrNecRxQueueLast,
};

typedef struct IrNecRxTaskParams
{
    QueueHandleVec_t out_ir_nec_frames;
} IrNecRxTaskParams_t;

void ir_nec_task_main(void*);

#endif //T2V_MOD_FW_IR_NEC_RX_TASK_H