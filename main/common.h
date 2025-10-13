//
// Created by felix on 10.10.25.
//

#ifndef T2V_MODULE_FW_COMMON_H
#define T2V_MODULE_FW_COMMON_H

#define FW_VERSION_MAJOR (0)
#define FW_VERSION_MINOR (1)
#define FW_VERSION_PATCH (0)

#define TAG_T2V_MODULE "tv2_module"
#define TAG_T2V_MODULE_NEC_RCV "t2v_module::nec"
#define TAG_T2V_MODULE_NEC_DECODER "t2v_module::nec::decoder"
#define TAG_T2V_MODULE_USB "t2v_module::usb"
#include <stdint.h>

#include "esp_cpu.h"


// Entry points for tasks

void ir_nec_task_main(void*);
void usb_device_task_main(void*);

// Global state

extern uint32_t address_config;

__always_inline uint32_t read_address_config()
{
    uint32_t local_address_config;
    do
    {
        local_address_config = address_config;
    }
    while (!esp_cpu_compare_and_set(&address_config, local_address_config, local_address_config));
    return local_address_config;
}

__always_inline uint8_t read_unicast_address()
{
    uint32_t local_address_config = read_address_config();

    return (uint8_t)(local_address_config & 0xff);
}

__always_inline void write_unicast_address(uint8_t address)
{
    uint32_t local_address_config;
    uint32_t new_address_config;
    do
    {
        local_address_config = address_config;
        new_address_config = (local_address_config & ~0xff) | (uint32_t)address;
    }
    while (esp_cpu_compare_and_set(&address_config, local_address_config, new_address_config));
}

__always_inline uint16_t read_multicast_mask()
{
    uint32_t local_address_config = read_address_config();

    return (uint16_t)(local_address_config & 0xffff << 8) >> 8;
}

__always_inline void write_multicast_mask(uint16_t multicast_mask)
{
    uint32_t local_address_config;
    uint32_t new_address_config;
    do
    {
        local_address_config = address_config;
        new_address_config = (local_address_config & ~(0xffff << 8)) | (uint32_t)multicast_mask << 8;
    }
    while (esp_cpu_compare_and_set(&address_config, local_address_config, new_address_config));
}

#endif //T2V_MODULE_FW_COMMON_H
