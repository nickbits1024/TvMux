#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "wifi.h"
#include "ble_int.h"
#include "ble.h"

#define TAG "BLE"

#define BLE_DEVICE_NAME "SmartSpeaker"

#define ESP_APP_ID                  0x55
#define SVC_INST_ID                 0
#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

enum
{
    WIFI_SVC_IDX,
    WIFI_SSID_CHAR_IDX,
    WIFI_SSID_CHAR_VAL_IDX,

    WIFI_PASSWORD_CHAR_IDX,
    WIFI_PASSWORD_CHAR_VAL_IDX,

    WIFI_IDX_NUM,
};

static uint8_t raw_adv_data[] = {
    /* flags */
    0x02, 0x01, 0x06,
    /* tx power*/
    0x02, 0x0a, 0xeb,
    /* service uuid */
    0x03, 0x03, 0xFF, 0x00,
    /* device name */
    0x06, 0x09, 'T', 'v', 'M', 'u', 'x'
};
static uint8_t raw_scan_rsp_data[] = {
    /* flags */
    0x02, 0x01, 0x06,
    /* tx power */
    0x02, 0x0a, 0xeb,
    /* service uuid */
    0x11, 0x06, 0xD8, 0x5F, 0x9E, 0x36, 0xEA, 0x11, 0x41, 0xFD, 0xB2, 0x85, 0x43, 0x3D, 0x91, 0xDD, 0x62, 0xEE
};

static const uint8_t wifi_service_guid[] = { 0xD8, 0x5F, 0x9E, 0x36, 0xEA, 0x11, 0x41, 0xFD, 0xB2, 0x85, 0x43, 0x3D, 0x91, 0xDD, 0x62, 0xEE };
static const uint8_t wifi_password_guid[] = { 0x01, 0x2A, 0xDB, 0x5E, 0x98, 0x26, 0x4C, 0x9C, 0xB5, 0x04, 0x79, 0xBB, 0xD0, 0xC0, 0x57, 0x48 }; 
static const uint8_t wifi_ssid_guid[] = { 0x29, 0x16, 0xD4, 0x9D, 0x7D, 0xAC, 0x4B, 0x26, 0xB8, 0xB7, 0x88, 0x8E, 0xC7, 0x60, 0x62, 0x14 }; 

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = BLE_AD_INTERVAL,
    .adv_int_max = BLE_AD_INTERVAL,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static uint8_t adv_config_done = 0;
uint16_t wifi_handle_table[WIFI_IDX_NUM];

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
//static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
//static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;
//static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

static uint8_t wifi_ssid[WIFI_SSID_SIZE];
static uint8_t wifi_password[WIFI_PASSWORD_SIZE];

static esp_gatts_attr_db_t gatt_db[WIFI_IDX_NUM] =
{
    // Service Declaration
    [WIFI_SVC_IDX] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&primary_service_uuid, ESP_GATT_PERM_READ,
      ESP_UUID_LEN_128, ESP_UUID_LEN_128, (uint8_t*)wifi_service_guid}},

    /* Characteristic Declaration */
    [WIFI_SSID_CHAR_IDX] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_read_write}},

    /* Characteristic Value */
    [WIFI_SSID_CHAR_VAL_IDX] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t*)wifi_ssid_guid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(wifi_ssid) - 1, 0, (uint8_t*)wifi_ssid}},

    /* Characteristic Declaration */
    [WIFI_PASSWORD_CHAR_IDX] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_write}},

    /* Characteristic Value */
    [WIFI_PASSWORD_CHAR_VAL_IDX] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t*)wifi_password_guid, ESP_GATT_PERM_WRITE,
      sizeof(wifi_password) - 1, 0, (uint8_t*)wifi_password}}
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param)
{
    switch (event)
    {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0)
            {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
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
                ESP_LOGE(TAG, "advertising start failed");
            }
            else
            {
                ESP_LOGI(TAG, "advertising start successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGE(TAG, "Advertising stop failed");
            }
            else
            {
                ESP_LOGI(TAG, "Stop adv successfully");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                param->update_conn_params.status,
                param->update_conn_params.min_int,
                param->update_conn_params.max_int,
                param->update_conn_params.conn_int,
                param->update_conn_params.latency,
                param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param)
{
    switch (event)
    {
        case ESP_GATTS_REG_EVT:
        {
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(BLE_DEVICE_NAME);
            if (set_dev_name_ret)
            {
                ESP_LOGE(TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
            if (raw_adv_ret)
            {
                ESP_LOGE(TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
            if (raw_scan_ret)
            {
                ESP_LOGE(TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, WIFI_IDX_NUM, SVC_INST_ID);
            if (create_attr_ret)
            {
                ESP_LOGE(TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
            //printf("ssid: %s\n", wifi_ssid);

            // esp_err_t attr_ret = esp_ble_gatts_set_attr_value(wifi_handle_table[WIFI_SSID_CHAR_VAL_IDX], strlen((const char*)wifi_ssid), wifi_ssid);
            // if (attr_ret != ESP_OK)
            // {
            //     ESP_LOGE(TAG, "esp_ble_gatts_set_attr_value failed (%s)", esp_err_to_name(attr_ret));
            // }
        }
        break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_READ_EVT");
            break;
        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep)
            {
                ESP_LOGI(TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
                if (wifi_handle_table[WIFI_SSID_CHAR_VAL_IDX] == param->write.handle && param->write.len < sizeof(wifi_ssid) - 1)
                {
                    memcpy(wifi_ssid, param->write.value, param->write.len);
                    wifi_ssid[param->write.len] = 0;
                    wifi_set_ssid((const char*)wifi_ssid);
                }
                else if (wifi_handle_table[WIFI_PASSWORD_CHAR_VAL_IDX] == param->write.handle && param->write.len < sizeof(wifi_password) - 1)
                {   
                    memcpy(wifi_password, param->write.value, param->write.len);
                    wifi_password[param->write.len] = 0;
                    wifi_set_password((const char*)wifi_password);
                }
                else
                {
                    esp_log_buffer_hex(TAG, param->write.value, param->write.len);
                }

                if (param->write.need_rsp)
                {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }
            else
            {
                ESP_LOGI(TAG, "ESP_GATTS_WRITE_EVT prepare");
            }   
            break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
            ESP_LOGI(TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            esp_log_buffer_hex(TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = { 0 };
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        {
            if (param->add_attr_tab.status != ESP_GATT_OK)
            {
                ESP_LOGE(TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != WIFI_IDX_NUM)
            {
                ESP_LOGE(TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, WIFI_IDX_NUM);
            }
            else
            {
                ESP_LOGI(TAG, "create attribute table successfully, the number handle = %d\n", param->add_attr_tab.num_handle);
                memcpy(wifi_handle_table, param->add_attr_tab.handles, sizeof(wifi_handle_table));
                esp_ble_gatts_start_service(wifi_handle_table[WIFI_SVC_IDX]);
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


esp_err_t ble_init()
{
    ESP_RETURN_ON_ERROR(wifi_get_ssid((char*)wifi_ssid, sizeof(wifi_ssid)), TAG, "wifi_get_ssid failed (%s)", esp_err_to_name(err_rc_));
    gatt_db[WIFI_SSID_CHAR_VAL_IDX].att_desc.length = (uint16_t)strlen((const char*)wifi_ssid);

    ESP_LOGI(TAG, "load ssid %s", wifi_ssid);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    //esp_bt_mode_t mode = ESP_BT_MODE_BTDM;
    ESP_RETURN_ON_ERROR(esp_bt_controller_init(&bt_cfg), TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(err_rc_));
    ESP_RETURN_ON_ERROR(esp_bt_controller_enable(ESP_BT_MODE_BTDM), TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(err_rc_));
    ESP_RETURN_ON_ERROR(esp_bluedroid_init(), TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(err_rc_));
    ESP_RETURN_ON_ERROR(esp_bluedroid_enable(), TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(err_rc_));
    ESP_RETURN_ON_ERROR(esp_ble_gatts_register_callback(gatts_profile_event_handler), TAG, "gatts register error (%s)", esp_err_to_name(err_rc_));
    ESP_RETURN_ON_ERROR(esp_ble_gap_register_callback(gap_event_handler), TAG, "gap register error (%s)", esp_err_to_name(err_rc_));
    ESP_RETURN_ON_ERROR(esp_ble_gatts_app_register(ESP_APP_ID), TAG, "gatts app register error (%s)", esp_err_to_name(err_rc_));
    ESP_RETURN_ON_ERROR(esp_ble_gatt_set_local_mtu(500), TAG, "set local  MTU failed, (%s)", esp_err_to_name(err_rc_));

    return ESP_OK;
}