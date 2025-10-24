//
// Created by felix on 20.10.25.
//

#include "FreeRTOS.h"
#include "common.h"
#include "ble_task.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_hosted_bluedroid.h"

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "portmacro.h"
#include "task.h"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define ESP_APP_ID                  0x55
#define SVC_INST_ID                 0

#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

static uint8_t adv_config_done = 0;

static bool notify_ir_data = false;
static esp_gatt_if_t notify_ir_gatts_if;
static uint16_t notify_ir_conn_id;

typedef struct
{
    uint8_t* prepare_buf;
    int prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

static uint8_t service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0x9b, 0xfe, 0xce, 0xbd, 0x5c, 0xaf, 0x4d, 0x90, 0x94, 0x2a, 0x09, 0x18, 0x00, 0xd4, 0x02, 0xc9
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //test_manufacturer,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};


static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst t2v_module_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    }
};

#define CHAR_TEAM_NAME_MAX_LENGTH (50)
#define CHAR_CAR_NAME_MAX_LENGTH (50)
#define CHAR_UNICAST_MAX_LENGTH (1)
#define CHAR_MULTICAST_MAX_LENGTH (2)

/* Service */
static const uint16_t CHAR_TEAM_NAME = 0xFF01;
static const uint16_t CHAR_CAR_NAME = 0xFF02;
static const uint16_t CHAR_3_SHORT_NOTIFY = 0xFF03;
static const uint16_t CHAR_UNICAST_WR = 0xFF04;
static const uint16_t CHAR_MULTICAST_WR = 0xFF05;

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint16_t character_user_description = ESP_GATT_UUID_CHAR_DESCRIPTION;
static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_team_name_name[] = "Team Id";
static const uint8_t char_car_name_name[] = "Car Id";
static const uint8_t char_ir_data_name[] = "IR Data";
static const uint8_t char_unicast_name[] = "Unicast Address";
static const uint8_t char_multicast_name[] = "Multicast Mask";
static const uint8_t char_ccc[2] = {0x00, 0x00};
static const uint8_t team_name[] = "Scuderia Segfault";
static const uint8_t car_name[5] = "car7";
static uint8_t unicast_address = 0x10;
static uint16_t multicast_mask = 0x0008;

static uint16_t gatt_db_handle_table[LAST_IDX];

/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[LAST_IDX] =
{
    // Service Declaration
    [IDX_SVC] =
    {
        {ESP_GATT_AUTO_RSP}, {
            ESP_UUID_LEN_16, (uint8_t*)&primary_service_uuid, ESP_GATT_PERM_READ,
            sizeof(uint16_t), sizeof(service_uuid), service_uuid
        }
    },

    /* Characteristic Declaration */
    [IDX_CHAR_TEAM_ID] =
    {
        {ESP_GATT_AUTO_RSP}, {
            ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
            CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_read
        }
    },

    /* Characteristic Value */
    [IDX_CHAR_VAL_TEAM_ID] =
    {
        {ESP_GATT_AUTO_RSP}, {
            ESP_UUID_LEN_16, (uint8_t*)&CHAR_TEAM_NAME, ESP_GATT_PERM_READ,
            CHAR_TEAM_NAME_MAX_LENGTH, sizeof(team_name), (uint8_t*)team_name
        }
    },

    /* Characteristic User Descriptor */
    [IDX_CHAR_CFG_TEAM_ID] =
    {
        {ESP_GATT_AUTO_RSP}, {
            ESP_UUID_LEN_16, (uint8_t*)&character_user_description, ESP_GATT_PERM_READ,
            sizeof(char_team_name_name), sizeof(char_team_name_name), (uint8_t*)char_team_name_name
        }
    },

    /* Characteristic Declaration */
    [IDX_CHAR_CAR_ID] =
    {
        {ESP_GATT_AUTO_RSP}, {
            ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
            CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_read
        }
    },

    /* Characteristic Value */
    [IDX_CHAR_VAL_CAR_ID] =
    {
        {ESP_GATT_AUTO_RSP}, {
            ESP_UUID_LEN_16, (uint8_t*)&CHAR_CAR_NAME, ESP_GATT_PERM_READ,
            CHAR_CAR_NAME_MAX_LENGTH, sizeof(car_name), (uint8_t*)car_name
        }
    },

    /* Characteristic User Descriptor */
    [IDX_CHAR_CFG_CAR_ID] =
    {
        {ESP_GATT_AUTO_RSP}, {
            ESP_UUID_LEN_16, (uint8_t*)&character_user_description, ESP_GATT_PERM_READ,
            sizeof(char_car_name_name), sizeof(char_car_name_name), (uint8_t*)char_car_name_name
        }
    },

    /* Characteristic Declaration */
    [IDX_CHAR_UNICAST] =
    {
        {ESP_GATT_RSP_BY_APP}, {
            ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
            CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_read_write
        }
    },

    /* Characteristic Value */
    [IDX_CHAR_VAL_UNICAST] =
    {
        {ESP_GATT_AUTO_RSP}, {
            ESP_UUID_LEN_16, (uint8_t*)&CHAR_UNICAST_WR, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            CHAR_UNICAST_MAX_LENGTH, sizeof(unicast_address), &unicast_address
        }
    },

    /* Characteristic User Descriptor */
    [IDX_CHAR_CFG_UNICAST] =
    {
        {ESP_GATT_AUTO_RSP}, {
            ESP_UUID_LEN_16, (uint8_t*)&character_user_description, ESP_GATT_PERM_READ,
            sizeof(char_unicast_name), sizeof(char_unicast_name), (uint8_t*)char_unicast_name
        }
    },

    /* Characteristic Declaration */
    [IDX_CHAR_MULTICAST] =
    {
        {ESP_GATT_AUTO_RSP}, {
            ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
            CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_read_write
        }
    },

    /* Characteristic Value */
    [IDX_CHAR_VAL_MULTICAST] =
    {
        {ESP_GATT_RSP_BY_APP}, {
            ESP_UUID_LEN_16, (uint8_t*)&CHAR_MULTICAST_WR, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            CHAR_MULTICAST_MAX_LENGTH, sizeof(multicast_mask), (uint8_t*)&multicast_mask
        }
    },

    /* Characteristic User Descriptor */
    [IDX_CHAR_CFG_MULTICAST] =
    {
        {ESP_GATT_AUTO_RSP}, {
            ESP_UUID_LEN_16, (uint8_t*)&character_user_description, ESP_GATT_PERM_READ,
            sizeof(char_multicast_name), sizeof(char_multicast_name), (uint8_t*)char_multicast_name
        }
    },

    /* Characteristic Declaration */
    [IDX_CHAR_IR_DATA] =
    {
        {ESP_GATT_AUTO_RSP}, {
            ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
            CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_notify
        }
    },

    /* Characteristic Value */
    [IDX_CHAR_VAL_IR_DATA] =
    {
        {ESP_GATT_AUTO_RSP}, {
            ESP_UUID_LEN_16, (uint8_t*)&CHAR_3_SHORT_NOTIFY, 0,
            GATTS_CHAR_VAL_LEN_MAX, 0, NULL
        }
    },

    /* Characteristic User Descriptor */
    [IDX_CHAR_CFG_IR_DATA] =
    {
        {ESP_GATT_AUTO_RSP}, {
            ESP_UUID_LEN_16, (uint8_t*)&character_user_description, ESP_GATT_PERM_READ,
            sizeof(char_ir_data_name), sizeof(char_ir_data_name), (uint8_t*)char_ir_data_name
        }
    },

    /* Characteristic Client Configuration Descriptor */
    [IDX_CHAR_CFG_IR_DATA_2] =
    {
        {ESP_GATT_AUTO_RSP}, {
            ESP_UUID_LEN_16, (uint8_t*)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            sizeof(uint16_t), sizeof(char_ccc), (uint8_t*)char_ccc
        }
    },

};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        /* advertising start complete event to indicate advertising start successfully or failed */
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(TAG_T2V_MODULE_BLE, "advertising start failed");
        }
        else
        {
            ESP_LOGD(TAG_T2V_MODULE_BLE, "advertising start successfully");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(TAG_T2V_MODULE_BLE, "Advertising stop failed");
        }
        else
        {
            ESP_LOGD(TAG_T2V_MODULE_BLE, "Stop adv successfully");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGD(TAG_T2V_MODULE_BLE, "update connection params status = %d, conn_int = %d, latency = %d, timeout = %d",
                 param->update_conn_params.status,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

void prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t* prepare_write_env,
                             esp_ble_gatts_cb_param_t* param)
{
    ESP_LOGD(TAG_T2V_MODULE_BLE, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.offset > PREPARE_BUF_MAX_SIZE)
    {
        status = ESP_GATT_INVALID_OFFSET;
    }
    else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE)
    {
        status = ESP_GATT_INVALID_ATTR_LEN;
    }

    if (status == ESP_GATT_OK && prepare_write_env->prepare_buf == NULL)
    {
        prepare_write_env->prepare_buf = (uint8_t*)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL)
        {
            ESP_LOGE(TAG_T2V_MODULE_BLE, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    }

    /*send response when param->write.need_rsp is true */
    if (param->write.need_rsp)
    {
        esp_gatt_rsp_t* gatt_rsp = (esp_gatt_rsp_t*)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL)
        {
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id,
                                                                 status, gatt_rsp);
            if (response_err != ESP_OK)
            {
                ESP_LOGE(TAG_T2V_MODULE_BLE, "Send response error");
            }
            free(gatt_rsp);
        }
        else
        {
            ESP_LOGE(TAG_T2V_MODULE_BLE, "%s, malloc failed, and no resource to send response", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    }
    if (status != ESP_GATT_OK)
    {
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
           param->write.value,
           param->write.len);
    prepare_write_env->prepare_len += param->write.len;
}

uint8_t long_write[16] = {
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF
};

void exec_write_event_env(prepare_type_env_t* prepare_write_env, esp_ble_gatts_cb_param_t* param)
{
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf)
    {
        if (prepare_write_env->prepare_len == 256)
        {
            bool long_write_success = true;
            for (uint16_t i = 0; i < prepare_write_env->prepare_len; i++)
            {
                if (prepare_write_env->prepare_buf[i] != long_write[i % 16])
                {
                    long_write_success = false;
                    break;
                }
            }
            if (long_write_success)
            {
                ESP_LOGD(TAG_T2V_MODULE_BLE, "(4) ***** long write success ***** ");
            }
        }
    }
    else
    {
        ESP_LOGD(TAG_T2V_MODULE_BLE, "ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf)
    {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t* param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        {
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(DEVICE_NAME);
            if (set_dev_name_ret)
            {
                ESP_LOGE(TAG_T2V_MODULE_BLE, "set device name failed, error code = %x", set_dev_name_ret);
            }
            //config adv data
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret)
            {
                ESP_LOGE(TAG_T2V_MODULE_BLE, "config adv data failed, error code = %x", ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            //config scan response data
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret)
            {
                ESP_LOGE(TAG_T2V_MODULE_BLE, "config scan response data failed, error code = %x", ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, LAST_IDX, SVC_INST_ID);
            if (create_attr_ret)
            {
                ESP_LOGE(TAG_T2V_MODULE_BLE, "create attr table failed, error code = %x", create_attr_ret);
            }
        }
        break;
    case ESP_GATTS_READ_EVT:
        ESP_LOGD(TAG_T2V_MODULE_BLE, "ESP_GATTS_READ_EVT");
        if (gatt_db_handle_table[IDX_CHAR_VAL_UNICAST] == param->read.handle && param->read.need_rsp)
        {
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));

            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = 1;
            rsp.attr_value.value[0] = read_unicast_address();

            esp_err_t ret = esp_ble_gatts_send_response(
                gatts_if,
                param->read.conn_id,
                param->read.trans_id,
                ESP_GATT_OK,
                &rsp
                );

            if (ret)
            {
                ESP_LOGE(TAG_T2V_MODULE_BLE, "send response failed, error code = %x", ret);
            }
        } else if (gatt_db_handle_table[IDX_CHAR_VAL_MULTICAST] == param->read.handle && param->read.need_rsp)
        {
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));

            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = 2;
            uint16_t multicast_mask_2 = read_multicast_mask();
            rsp.attr_value.value[0] = ((uint8_t*)&multicast_mask_2)[0];
            rsp.attr_value.value[1] = ((uint8_t*)&multicast_mask_2)[1];

            esp_err_t ret = esp_ble_gatts_send_response(
                gatts_if,
                param->read.conn_id,
                param->read.trans_id,
                ESP_GATT_OK,
                &rsp
                );

            if (ret)
            {
                ESP_LOGE(TAG_T2V_MODULE_BLE, "send response failed, error code = %x", ret);
            }
        }
        break;
    case ESP_GATTS_WRITE_EVT:
        if (!param->write.is_prep)
        {
            // the data length of gattc write  must be less than GATTS_EXAMPLE_CHAR_VAL_LEN_MAX.
            if (gatt_db_handle_table[IDX_CHAR_CFG_IR_DATA_2] == param->write.handle && param->write.len == 2)
            {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                uint8_t notify_data[2];
                notify_data[0] = 0xAA;
                notify_data[1] = 0xBB;

                if (descr_value == 0x0001)
                {
                    //the size of notify_data[] need less than MTU size
                    /*esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gatt_db_handle_table[IDX_CHAR_VAL_IR_DATA],
                                            sizeof(notify_data), notify_data, false);*/
                    notify_ir_data = true;
                    notify_ir_gatts_if = gatts_if;
                    notify_ir_conn_id = param->write.conn_id;
                    ESP_LOGD(TAG_T2V_MODULE_BLE, "enabling IR data notifications");
                }
                else if (descr_value == 0x0002)
                {
                    //the size of indicate_data[] need less than MTU size
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id,
                                                gatt_db_handle_table[IDX_CHAR_VAL_IR_DATA],
                                                sizeof(notify_data), notify_data, true);
                }
                else if (descr_value == 0x0000)
                {
                    notify_ir_data = false;
                    ESP_LOGD(TAG_T2V_MODULE_BLE, "disabling IR data notifications");
                }
                else
                {
                    ESP_LOGE(TAG_T2V_MODULE_BLE, "unknown descr value");
                    ESP_LOG_BUFFER_HEX(TAG_T2V_MODULE_BLE, param->write.value, param->write.len);
                }
            }

            if (gatt_db_handle_table[IDX_CHAR_VAL_UNICAST] == param->write.handle && param->write.len == 1)
            {
                unicast_address = param->write.value[0];
                write_unicast_address(unicast_address);
                ESP_LOGI(TAG_T2V_MODULE_BLE, "updated unicast address: %02x", param->write.value[0]);
            }
            else if (gatt_db_handle_table[IDX_CHAR_VAL_MULTICAST] == param->write.handle && param->write.len == 2)
            {
                multicast_mask = *(uint16_t*)param->write.value;
                write_multicast_mask(multicast_mask);
                ESP_LOGI(TAG_T2V_MODULE_BLE, "updated multicast mask: %04x", *(uint16_t*)param->write.value);
            }

            /* send response when param->write.need_rsp is true*/
            if (param->write.need_rsp)
            {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            }
        }
        else
        {
            /* handle prepare write */
            prepare_write_event_env(gatts_if, &prepare_write_env, param);
        }
        break;
    case ESP_GATTS_EXEC_WRITE_EVT:
        // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
        ESP_LOGD(TAG_T2V_MODULE_BLE, "ESP_GATTS_EXEC_WRITE_EVT");
        exec_write_event_env(&prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGD(TAG_T2V_MODULE_BLE, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGD(TAG_T2V_MODULE_BLE, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status,
                 param->conf.handle);
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGD(TAG_T2V_MODULE_BLE, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status,
                 param->start.service_handle);
        break;
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG_T2V_MODULE_BLE, "Device connected with conn_id %d", param->connect.conn_id);
        //ESP_LOG_BUFFER_HEX(TAG_T2V_MODULE_BLE, param->connect.remote_bda, 6);
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20; // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10; // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400; // timeout = 400*10ms = 4000ms
        //start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG_T2V_MODULE_BLE, "Device disconnected reason 0x%x, starting advertising", param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        {
            if (param->add_attr_tab.status != ESP_GATT_OK)
            {
                ESP_LOGE(TAG_T2V_MODULE_BLE, "create attribute table failed, error code=0x%x",
                         param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != LAST_IDX)
            {
                ESP_LOGE(TAG_T2V_MODULE_BLE, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, LAST_IDX);
            }
            else
            {
                ESP_LOGD(TAG_T2V_MODULE_BLE, "create attribute table successfully, the number handle = %d",
                         param->add_attr_tab.num_handle);
                memcpy(gatt_db_handle_table, param->add_attr_tab.handles, sizeof(gatt_db_handle_table));
                esp_ble_gatts_start_service(gatt_db_handle_table[IDX_SVC]);
            }
            break;
        }
    case ESP_GATTS_STOP_EVT:
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    case ESP_GATTS_UNREG_EVT:
    case ESP_GATTS_DELETE_EVT:
    default:
        break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            t2v_module_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGE(TAG_T2V_MODULE_BLE, "reg app failed, app_id %04x, status %d",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }
    do
    {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == t2v_module_profile_tab[idx].gatts_if)
            {
                if (t2v_module_profile_tab[idx].gatts_cb)
                {
                    t2v_module_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    }
    while (0);
}


void ble_task_main(void* task_params)
{
    QueueHandle_t* ir_data_queue = task_params;

    esp_err_t ret;

    /* Initialize NVS. */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* initialize TRANSPORT first */
    hosted_hci_bluedroid_open();

    /* get HCI driver operations */
    esp_bluedroid_hci_driver_operations_t operations = {
        .send = hosted_hci_bluedroid_send,
        .check_send_available = hosted_hci_bluedroid_check_send_available,
        .register_host_callback = hosted_hci_bluedroid_register_host_callback,
    };
    ret = esp_bluedroid_attach_hci_driver(&operations);
    if (ret)
    {
        ESP_LOGE(TAG_T2V_MODULE_BLE, "%s attaching hci driver failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(TAG_T2V_MODULE_BLE, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(TAG_T2V_MODULE_BLE, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }


    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret)
    {
        ESP_LOGE(TAG_T2V_MODULE_BLE, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret)
    {
        ESP_LOGE(TAG_T2V_MODULE_BLE, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (ret)
    {
        ESP_LOGE(TAG_T2V_MODULE_BLE, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret)
    {
        ESP_LOGE(TAG_T2V_MODULE_BLE, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    ESP_LOGI(TAG_T2V_MODULE_BLE, "finished initializing BLE config interface");
    uint8_t data[4];

    while (1)
    {
        if (xQueueReceive(*ir_data_queue, &data, pdMS_TO_TICKS(10)))
        {
            if (notify_ir_data)
            {
                esp_ble_gatts_send_indicate(notify_ir_gatts_if, notify_ir_conn_id, gatt_db_handle_table[IDX_CHAR_VAL_IR_DATA],
                                                sizeof(data), data, true);
            }
        }
    }
}
