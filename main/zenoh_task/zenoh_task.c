//
// Created by felix on 12.11.25.
//

#include "../t2v_types.h"
#include "zenoh_task.h"

#include <FreeRTOS.h>
#include <queue.h>
#include <freertos/task.h>

#include "esp_log.h"
#include "zenoh-pico/api/macros.h"
#include "zenoh-pico/api/primitives.h"
#include "zenoh-pico/api/types.h"

static uint16_t caps = T2V_ZENOH_CAPABILITIES;

#ifdef CONFIG_T2V_CAP_SEND_IR_FRAMES
QueueHandle_t ir_sender_queue;
#endif

void query_handler(z_loaned_query_t* query, void* ctx)
{
    z_owned_bytes_t reply_payload;
    z_bytes_from_buf(&reply_payload, (uint8_t*)&caps, sizeof(caps), NULL, ctx);

    z_query_reply(query, z_query_keyexpr(query), z_move(reply_payload), NULL);
}

#ifdef CONFIG_T2V_CAP_SEND_IR_FRAMES
void ir_sender_handler(z_loaned_sample_t* sample, void* ctx)
{
    z_owned_slice_t data;
    z_bytes_to_slice(z_sample_payload(sample), &data);

    if (data._val.len != 4)
    {
        ESP_LOGW(TAG_T2V_MODULE_ZENOH, "Data length mismatch");
    }
    else
    {
        ESP_LOG_BUFFER_HEX(TAG_T2V_MODULE_ZENOH, z_slice_data(z_loan(data)), z_slice_len(z_loan(data)));
        uint8_t buffer[4];
        memcpy(buffer, z_slice_data(z_loan(data)), sizeof(buffer));

        xQueueSendToBack(ir_sender_queue, buffer, pdMS_TO_TICKS(100));
    }

    z_slice_drop(z_slice_move(&data));
}
#endif

struct zenoh_state
{
    z_owned_queryable_t caps_queryable;
#ifdef CONFIG_T2V_CAP_SEND_IR_FRAMES
    z_owned_subscriber_t ir_subscriber;
    z_owned_subscriber_t ir_ip_subscriber;
#endif
};

void init_session(z_loaned_session_t* s, struct zenoh_state* state, const z_loaned_keyexpr_t* base_ke,
                  const z_loaned_keyexpr_t* device_base_ke)
{
    zp_start_read_task(s, NULL);
    zp_start_lease_task(s, NULL);

    z_view_keyexpr_t caps_endpoint;
    z_view_keyexpr_from_str_unchecked(&caps_endpoint, "caps");
#ifdef CONFIG_T2V_CAP_SEND_IR_FRAMES
    z_view_keyexpr_t ir_endpoint;
    z_view_keyexpr_from_str_unchecked(&ir_endpoint, "ir");
#endif

    z_owned_keyexpr_t caps_ke;
    z_keyexpr_join(&caps_ke, device_base_ke, z_view_keyexpr_loan(&caps_endpoint));

    z_owned_closure_query_t caps_callback;
    z_closure(&caps_callback, query_handler, NULL, NULL);
    if (z_declare_queryable(s, &state->caps_queryable, z_loan(caps_ke), z_move(caps_callback), NULL) < 0)
    {
        printf("Unable to declare queryable.\n");
        exit(-1);
    }

#ifdef CONFIG_T2V_CAP_SEND_IR_FRAMES
    z_owned_keyexpr_t ir_ke;
    z_keyexpr_join(&ir_ke, base_ke, z_view_keyexpr_loan(&ir_endpoint));

    z_owned_closure_sample_t ir_callback;
    z_closure(&ir_callback, ir_sender_handler, NULL, NULL);
    if (z_declare_subscriber(s, &state->ir_subscriber, z_loan(ir_ke), z_move(ir_callback), NULL) < 0)
    {
        printf("Unable to declare subscriber.\n");
        exit(-1);
    }

    z_owned_keyexpr_t ir_ip_ke;
    z_keyexpr_join(&ir_ip_ke, device_base_ke, z_view_keyexpr_loan(&ir_endpoint));

    z_owned_closure_sample_t ir_ip_callback;
    z_closure(&ir_ip_callback, ir_sender_handler, NULL, NULL);
    if (z_declare_subscriber(s, &state->ir_ip_subscriber, z_loan(ir_ip_ke), z_move(ir_ip_callback), NULL) < 0)
    {
        printf("Unable to declare subscriber.\n");
        exit(-1);
    }
    z_keyexpr_drop(z_move(ir_ke));
    z_keyexpr_drop(z_move(ir_ip_ke));
#endif
    z_keyexpr_drop(z_move(caps_ke));
}

void zenoh_task_main(void* params)
{
    ZenohTaskParams_t* task_params = params;
    NicStatus_t nic_status;
    struct esp_ip4_addr ip_addr;
#ifdef CONFIG_T2V_LED_DRIVER_ENABLE
    LedUpdate_t led_update;

    led_update.flags = LED_UPDATE_LEDS_VALID;
    led_update.leds = 0x11;
    led_update.leds_mask = 0xff;

    xQueueSendToBack(task_params->out_led_driver, &led_update, pdMS_TO_TICKS(100));
#endif

#ifdef CONFIG_T2V_CAP_SEND_IR_FRAMES
    ir_sender_queue = task_params->out_ir_frame;
#endif

    ESP_LOGI(TAG_T2V_MODULE_ZENOH, "Macro derived locator: %s", T2V_ZENOH_LOCATOR);

    while (1)
    {
        if (xQueueReceive(task_params->in_nic_status, &nic_status, portMAX_DELAY))
        {
#ifdef CONFIG_T2V_LED_DRIVER_ENABLE
            if (nic_status.status == NIC_STATUS_CONNECTED)
            {
                led_update.leds = 0x22;
                xQueueSendToBack(task_params->out_led_driver, &led_update, pdMS_TO_TICKS(100));
            }
            else
#endif
            if (nic_status.status == NIC_STATUS_GOT_IP)
            {
                ip_addr = nic_status.payload.ip_addr;
#ifdef CONFIG_T2V_LED_DRIVER_ENABLE
                led_update.leds = 0x44;
                xQueueSendToBack(task_params->out_led_driver, &led_update, pdMS_TO_TICKS(100));
#endif
                break;
            }
        }
    }

#if ZENOH_ESPIDF == 1
    z_owned_config_t config;
    z_config_default(&config);
    zp_config_insert(z_loan_mut(config), Z_CONFIG_MODE_KEY, Z_CONFIG_MODE_CLIENT);
    zp_config_insert(z_loan_mut(config), Z_CONFIG_CONNECT_KEY, T2V_ZENOH_LOCATOR);

    char ip_addr_str[9];
    ip_addr_str[0] = 0;
    sprintf(ip_addr_str, "%02X%02X%02X%02X", *(uint8_t*)&ip_addr.addr, *((uint8_t*)&ip_addr.addr + 1),
            *((uint8_t*)&ip_addr.addr + 2), *((uint8_t*)&ip_addr.addr + 3));

    ESP_LOGI(TAG_T2V_MODULE_ZENOH, "IP derived device ID: %s", ip_addr_str);

    z_view_keyexpr_t base_ke;
    z_view_keyexpr_from_str_unchecked(&base_ke, CONFIG_T2V_ZENOH_BASE_KEYEXPR);

    z_view_keyexpr_t ip_ke;
    z_view_keyexpr_from_str_unchecked(&ip_ke, ip_addr_str);
    z_owned_keyexpr_t device_base_ke;
    z_keyexpr_join(&device_base_ke, z_view_keyexpr_loan(&base_ke), z_view_keyexpr_loan(&ip_ke));

    struct zenoh_state state;

    z_owned_session_t s;
    while (1)
    {
        if (z_open(&s, z_move(config), NULL) < 0)
        {
            ESP_LOGE(TAG_T2V_MODULE_ZENOH, "Connection to Zenoh failed. Retry in 5 sec...");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
        else
        {
            break;
        }
    }

    init_session(z_loan_mut(s), &state, z_loan(base_ke), z_loan(device_base_ke));

#ifdef CONFIG_T2V_LED_DRIVER_ENABLE
    led_update.leds = 0xCC;
    xQueueSendToBack(task_params->out_led_driver, &led_update, pdMS_TO_TICKS(100));
#endif

    while (1)
    {
        if (xQueueReceive(task_params->in_nic_status, &nic_status, portMAX_DELAY))
        {
            if (nic_status.status == NIC_STATUS_DISCONNECTED)
            {
#ifdef CONFIG_T2V_CAP_SEND_IR_FRAMES
                z_subscriber_drop(z_move(state.ir_ip_subscriber));
                z_subscriber_drop(z_move(state.ir_subscriber));
#endif
                z_queryable_drop(z_move(state.caps_queryable));
                z_session_drop(z_move(s));
#ifdef CONFIG_T2V_LED_DRIVER_ENABLE
                led_update.leds = 0x11;
                xQueueSendToBack(task_params->out_led_driver, &led_update, pdMS_TO_TICKS(100));
#endif
            }
#ifdef CONFIG_T2V_LED_DRIVER_ENABLE
            else if (nic_status.status == NIC_STATUS_CONNECTED)
            {
                led_update.leds = 0x22;
                xQueueSendToBack(task_params->out_led_driver, &led_update, pdMS_TO_TICKS(100));
            }
#endif
            else if (nic_status.status == NIC_STATUS_GOT_IP)
            {
#ifdef CONFIG_T2V_LED_DRIVER_ENABLE
                led_update.leds = 0x44;
                xQueueSendToBack(task_params->out_led_driver, &led_update, pdMS_TO_TICKS(100));
#endif

                while (1)
                {
                    if (z_open(&s, z_move(config), NULL) < 0)
                    {
                        ESP_LOGE(TAG_T2V_MODULE_ZENOH, "Connection to Zenoh failed. Retry in 5 sec...");
                        vTaskDelay(pdMS_TO_TICKS(5000));
                    }
                    else
                    {
                        break;
                    }
                }

                ip_addr_str[0] = 0;
                sprintf(ip_addr_str, "%02X%02X%02X%02X", *(uint8_t*)&ip_addr.addr, *((uint8_t*)&ip_addr.addr + 1),
                        *((uint8_t*)&ip_addr.addr + 2), *((uint8_t*)&ip_addr.addr + 3));

                ESP_LOGI(TAG_T2V_MODULE_ZENOH, "IP derived device ID: %s", ip_addr_str);

                z_view_keyexpr_from_str_unchecked(&ip_ke, ip_addr_str);
                z_keyexpr_join(&device_base_ke, z_view_keyexpr_loan(&base_ke), z_view_keyexpr_loan(&ip_ke));

                init_session(z_loan_mut(s), &state, z_loan(base_ke), z_loan(device_base_ke));

#ifdef CONFIG_T2V_LED_DRIVER_ENABLE
                led_update.leds = 0xCC;
                xQueueSendToBack(task_params->out_led_driver, &led_update, pdMS_TO_TICKS(100));
#endif
            }
        }
    }

#else
    ESP_LOGE(TAG_T2V_MODULE_ZENOH, "Wrong Zenoh platform enabled");
#endif
}
