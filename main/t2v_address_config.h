//
// Created by felix on 17.11.25.
//

#ifndef T2V_MOD_FW_ADDRESS_CONFIG_H
#define T2V_MOD_FW_ADDRESS_CONFIG_H

#include <stdint.h>
#include <sys/cdefs.h>

#include "esp_cpu.h"
#include "esp_log.h"
#include "nvs.h"
#include "FreeRTOSConfig.h"

#ifdef CONFIG_T2V_ADDRESS_CONFIG_ENABLE
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

__always_inline void write_address_config_bare(uint32_t new_address_config)
{
    while (!esp_cpu_compare_and_set(&address_config, address_config, new_address_config))
    {
    }
}

__always_inline esp_err_t write_address_config(nvs_handle_t nvs_handle, uint32_t new_address_config)
{
    write_address_config_bare(new_address_config);
    return nvs_set_u32(nvs_handle, CONFIG_T2V_ADDRESS_CONFIG_NVS_KEY, new_address_config);
}

__always_inline void write_address_config_to_nvs(uint32_t new_address_config)
{
    nvs_handle_t nvs_handle;
    esp_err_t esp_err = nvs_open(CONFIG_T2V_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (esp_err != ESP_OK)
    {
        ESP_LOGE(TAG_T2V_MODULE, "Error (%s) opening NVS handle!", esp_err_to_name(esp_err));
        return;
    }

    esp_err = nvs_set_u32(nvs_handle, CONFIG_T2V_ADDRESS_CONFIG_NVS_KEY, new_address_config);
    if (esp_err != ESP_OK)
    {
        ESP_LOGE(TAG_T2V_MODULE, "Error (%s) opening NVS handle!", esp_err_to_name(esp_err));
        return;
    }

    esp_err = nvs_commit(nvs_handle);
    if (esp_err != ESP_OK)
    {
        ESP_LOGE(TAG_T2V_MODULE, "Error (%s) opening NVS handle!", esp_err_to_name(esp_err));
        return;
    }

    nvs_close(nvs_handle);
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
        ESP_LOGD(TAG_T2V_MODULE, "Writing address config old=%08x, new=%08x", local_address_config,
                 new_address_config);
    }
    while (!esp_cpu_compare_and_set(&address_config, local_address_config, new_address_config));
    write_address_config_to_nvs(new_address_config);
}

__always_inline uint16_t read_multicast_mask()
{
    uint32_t local_address_config = read_address_config();

    return (uint16_t)((local_address_config & 0xffff << 8) >> 8);
}

__always_inline void write_multicast_mask(uint16_t multicast_mask)
{
    uint32_t local_address_config;
    uint32_t new_address_config;
    do
    {
        local_address_config = address_config;
        new_address_config = (local_address_config & ~(0xffff << 8)) | ((uint32_t)multicast_mask) << 8;
        ESP_LOGD(TAG_T2V_MODULE, "Writing address config old=%08x, new=%08x", local_address_config,
                 new_address_config);
    }
    while (!esp_cpu_compare_and_set(&address_config, local_address_config, new_address_config));
    write_address_config_to_nvs(new_address_config);
}
#endif

#endif //T2V_MOD_FW_ADDRESS_CONFIG_H