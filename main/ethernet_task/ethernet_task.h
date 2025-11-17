//
// Created by felix on 16.11.25.
//

#ifndef T2V_MOD_FW_ETHERNET_TASK_H
#define T2V_MOD_FW_ETHERNET_TASK_H

#include "FreeRTOS.h"
#include "queue.h"
#include "../t2v_config.h"
#include "../t2v_types.h"

enum EthernetTaskQueues
{
#ifdef CONFIG_T2V_ZENOH_ENABLE
    EthernetTaskQueueZenoh,
#endif
    EthernetTaskQueueLast
};

typedef struct EthernetTaskParams
{
    QueueHandleVec_t out_nic_status;
} EthernetTaskParams_t;

void ethernet_task_main(void*);

#endif //T2V_MOD_FW_ETHERNET_TASK_H