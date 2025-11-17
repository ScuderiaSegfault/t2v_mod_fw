//
// Created by felix on 20.10.25.
//

#ifndef T2V_MOD_FW_BLE_TASK_PRIV_H
#define T2V_MOD_FW_BLE_TASK_PRIV_H
#include "esp_gatts_api.h"

#define DEVICE_NAME                 CONFIG_T2V_BLE_ADV_NAME
#define TEAM_NAME                   CONFIG_T2V_BLE_TEAM_NAME
#define CAR_ID                      CONFIG_T2V_BLE_CAR_ID
#define GATTS_CHAR_VAL_LEN_MAX      500

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t* param);

enum
{
    IDX_SVC,
    IDX_CHAR_TEAM_ID,
    IDX_CHAR_VAL_TEAM_ID,
    IDX_CHAR_CFG_TEAM_ID,

    IDX_CHAR_CAR_ID,
    IDX_CHAR_VAL_CAR_ID,
    IDX_CHAR_CFG_CAR_ID,

    IDX_CHAR_UNICAST,
    IDX_CHAR_VAL_UNICAST,
    IDX_CHAR_CFG_UNICAST,

    IDX_CHAR_MULTICAST,
    IDX_CHAR_VAL_MULTICAST,
    IDX_CHAR_CFG_MULTICAST,

    IDX_CHAR_IR_DATA,
    IDX_CHAR_VAL_IR_DATA,
    IDX_CHAR_CFG_IR_DATA,
    IDX_CHAR_CFG_IR_DATA_2,

    LAST_IDX
};


#endif //T2V_MOD_FW_BLE_TASK_H
