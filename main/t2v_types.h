//
// Created by felix on 15.11.25.
//

#ifndef T2V_MOD_FW_T2V_TYPES_H
#define T2V_MOD_FW_T2V_TYPES_H

#include "esp_netif_ip_addr.h"
#include <stdint.h>
#include "queue_handle_vec.h"

#define NIC_STATUS_STARTED  (0x00)
#define NIC_STATUS_CONNECTED (0x01)
#define NIC_STATUS_GOT_IP (0x02)
#define NIC_STATUS_DISCONNECTED (0x03)

typedef struct NicStatus
{
    uint8_t status;
    union
    {
        struct esp_ip4_addr ip_addr;
    } payload;
} NicStatus_t;

#define LED_UPDATE_SEVEN_SEGMENT_1_VALID (0x01)
#define LED_UPDATE_SEVEN_SEGMENT_2_VALID (0x02)
#define LED_UPDATE_LEDS_VALID (0x04)

typedef struct __attribute__((packed)) LedUpdate
{
    uint8_t flags;
    uint8_t seven_segment_1;
    uint8_t seven_segment_2;
    uint8_t leds;
    uint8_t leds_mask;
} LedUpdate_t;

#define BUTTON_EVENT_NONE (0x00)
#define BUTTON_EVENT_PRESSED (0x01)
#define BUTTON_EVENT_RELEASED (0x02)

typedef struct __attribute__((packed)) ButtonUpdate {
    uint8_t btn0_event;
    uint8_t btn1_event;
} ButtonUpdate_t;

#define DEVICE_CAP_IR_SENDER (0x0001)
#define DEVICE_CAP_STARTING_LIGHTS (0x0002)

#endif //T2V_MOD_FW_T2V_TYPES_H