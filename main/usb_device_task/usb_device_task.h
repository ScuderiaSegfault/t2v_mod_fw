//
// Created by felix on 17.11.25.
//

#ifndef T2V_MOD_FW_USB_DEVICE_TASK_H
#define T2V_MOD_FW_USB_DEVICE_TASK_H

#include "FreeRTOS.h"
#include "queue.h"

#include "../t2v_config.h"
#include "../t2v_types.h"

typedef struct UsbDeviceTaskParams
{
    QueueHandle_t in_ir_data;
} UsbDeviceTaskParams_t;

void usb_device_task_main(void*);

#endif //T2V_MOD_FW_USB_DEVICE_TASK_H