#include "bluetooth.h"
#include "config.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

static const char *TAG = "BLE_LOCK";

// ============================================================================
// Custom 128-bit UUIDs
// ============================================================================

// Lock Service UUID: 12340001-1234-5678-1234-56789abcdef0
static uint8_t service_uuid128[16] = {
    0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12, 0x01, 0x00, 0x34, 0x12
};

// Status Characteristic UUID: 12340002-1234-5678-1234-56789abcdef0
static uint8_t status_char_uuid128[16] = {
    0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12, 0x02, 0x00, 0x34, 0x12
};

// Command Characteristic UUID: 12340003-1234-5678-1234-56789abcdef0
static uint8_t command_char_uuid128[16] = {
    0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12, 0x03, 0x00, 0x34, 0x12
};

// Progress Characteristic UUID: 12340004-1234-5678-1234-56789abcdef0
static uint8_t progress_char_uuid128[16] = {
    0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12, 0x04, 0x00, 0x34, 0x12
};

// ============================================================================
// BLE Configuration
// ============================================================================

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

#define CONN_INTERVAL_MIN           24   // 30ms
#define CONN_INTERVAL_MAX           24   // 30ms
#define CONN_LATENCY                0
#define CONN_TIMEOUT                400  // 4s

// Attribute indexes
enum {
    IDX_SVC,
    IDX_STATUS_CHAR,
    IDX_STATUS_VAL,
    IDX_STATUS_CFG,         // Client Characteristic Configuration
    IDX_COMMAND_CHAR,
    IDX_COMMAND_VAL,
    IDX_PROGRESS_CHAR,
    IDX_PROGRESS_VAL,
    IDX_PROGRESS_CFG,
    IDX_NB,
};

// ============================================================================
// BLE State
// ============================================================================

static uint16_t ble_conn_id = 0;
static uint16_t ble_gatts_if = 0;
static bool ble_connected = false;
static uint16_t attr_handle_table[IDX_NB];
static bool service_started = false;

static SemaphoreHandle_t ble_mutex = NULL;
static bool status_notify_enabled = false;
static bool progress_notify_enabled = false;

// Command handling
static lock_command_t pending_command = LOCK_CMD_NONE;
static bluetooth_cmd_callback_t cmd_callback = NULL;

// Advertising data
static uint8_t adv_config_done = 0;

static uint8_t raw_adv_data[] = {
    // Flags
    0x02, 0x01, 0x06,
    // Complete local name
    0x05, 0x09, 'L', 'o', 'c', 'k',
    // Complete list of 128-bit service UUIDs
    0x11, 0x07,
    0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12, 0x01, 0x00, 0x34, 0x12
};

static uint8_t raw_scan_rsp_data[] = {
    0x02, 0x01, 0x06,
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// ============================================================================
// GATT Database
// ============================================================================

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;

// Characteristic values
static uint8_t status_char_value = LOCK_STATUS_UNKNOWN;
static uint8_t command_char_value = LOCK_CMD_NONE;
static uint8_t progress_char_value = 0;
static uint8_t status_ccc[2] = {0x00, 0x00};
static uint8_t progress_ccc[2] = {0x00, 0x00};

static const esp_gatts_attr_db_t gatt_db[IDX_NB] = {
    // Service Declaration
    [IDX_SVC] = {{ESP_GATT_AUTO_RSP}, 
                 {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
                  sizeof(service_uuid128), sizeof(service_uuid128), service_uuid128}},

    // Status Characteristic Declaration
    [IDX_STATUS_CHAR] = {{ESP_GATT_AUTO_RSP}, 
                         {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                          sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},

    // Status Characteristic Value
    [IDX_STATUS_VAL] = {{ESP_GATT_AUTO_RSP}, 
                        {ESP_UUID_LEN_128, status_char_uuid128, ESP_GATT_PERM_READ,
                         sizeof(uint8_t), sizeof(status_char_value), &status_char_value}},

    // Status CCC Descriptor
    [IDX_STATUS_CFG] = {{ESP_GATT_AUTO_RSP}, 
                        {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, 
                         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                         sizeof(uint16_t), sizeof(status_ccc), status_ccc}},

    // Command Characteristic Declaration
    [IDX_COMMAND_CHAR] = {{ESP_GATT_AUTO_RSP}, 
                          {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                           sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_write}},

    // Command Characteristic Value
    [IDX_COMMAND_VAL] = {{ESP_GATT_RSP_BY_APP},  // We handle write in app
                         {ESP_UUID_LEN_128, command_char_uuid128, ESP_GATT_PERM_WRITE,
                          sizeof(uint8_t), sizeof(command_char_value), &command_char_value}},

    // Progress Characteristic Declaration
    [IDX_PROGRESS_CHAR] = {{ESP_GATT_AUTO_RSP}, 
                           {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                            sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read_notify}},

    // Progress Characteristic Value
    [IDX_PROGRESS_VAL] = {{ESP_GATT_AUTO_RSP}, 
                          {ESP_UUID_LEN_128, progress_char_uuid128, ESP_GATT_PERM_READ,
                           sizeof(uint8_t), sizeof(progress_char_value), &progress_char_value}},

    // Progress CCC Descriptor
    [IDX_PROGRESS_CFG] = {{ESP_GATT_AUTO_RSP}, 
                          {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, 
                           ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                           sizeof(uint16_t), sizeof(progress_ccc), progress_ccc}},
};

// ============================================================================
// GAP Event Handler
// ============================================================================

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0) {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;

        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0) {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;

        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising start failed");
            } else {
                ESP_LOGI(TAG, "📡 Advertising started as '%s'", BLE_DEVICE_NAME);
            }
            break;

        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Advertising stopped");
            }
            break;

        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGD(TAG, "Connection params updated");
            break;

        default:
            break;
    }
}

// ============================================================================
// GATTS Event Handler
// ============================================================================

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, 
                                esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "GATT server registered");
            ble_gatts_if = gatts_if;
            
            esp_ble_gap_set_device_name(BLE_DEVICE_NAME);
            
            esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
            adv_config_done |= ADV_CONFIG_FLAG;
            
            esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
            
            esp_err_t ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, IDX_NB, 0);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Create attr table failed: %s", esp_err_to_name(ret));
            }
            break;

        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            if (param->add_attr_tab.status == ESP_GATT_OK) {
                if (param->add_attr_tab.num_handle == IDX_NB) {
                    memcpy(attr_handle_table, param->add_attr_tab.handles, sizeof(attr_handle_table));
                    
                    if (!service_started) {
                        esp_ble_gatts_start_service(attr_handle_table[IDX_SVC]);
                        service_started = true;
                        ESP_LOGI(TAG, "✅ Lock service started");
                    }
                }
            } else {
                ESP_LOGE(TAG, "Create attribute table failed: 0x%x", param->add_attr_tab.status);
            }
            break;

        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "🔗 Client connected");
            
            ble_conn_id = param->connect.conn_id;
            ble_gatts_if = gatts_if;
            
            xSemaphoreTake(ble_mutex, portMAX_DELAY);
            ble_connected = true;
            xSemaphoreGive(ble_mutex);
            
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.min_int = CONN_INTERVAL_MIN;
            conn_params.max_int = CONN_INTERVAL_MAX;
            conn_params.latency = CONN_LATENCY;
            conn_params.timeout = CONN_TIMEOUT;
            esp_ble_gap_update_conn_params(&conn_params);
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "❌ Client disconnected");
            
            xSemaphoreTake(ble_mutex, portMAX_DELAY);
            ble_connected = false;
            status_notify_enabled = false;
            progress_notify_enabled = false;
            xSemaphoreGive(ble_mutex);
            
            esp_ble_gap_start_advertising(&adv_params);
            break;

        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep) {
                // Handle Command write
                if (param->write.handle == attr_handle_table[IDX_COMMAND_VAL]) {
                    if (param->write.len >= 1) {
                        lock_command_t cmd = (lock_command_t)param->write.value[0];
                        ESP_LOGI(TAG, "📨 Command received: %d", cmd);
                        
                        xSemaphoreTake(ble_mutex, portMAX_DELAY);
                        pending_command = cmd;
                        xSemaphoreGive(ble_mutex);
                        
                        // Call callback if registered
                        if (cmd_callback != NULL) {
                            cmd_callback(cmd);
                        }
                    }
                    
                    // Send response
                    if (param->write.need_rsp) {
                        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, 
                                                   param->write.trans_id, ESP_GATT_OK, NULL);
                    }
                }
                
                // Handle Status CCC write
                else if (param->write.handle == attr_handle_table[IDX_STATUS_CFG] && 
                         param->write.len == 2) {
                    uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                    xSemaphoreTake(ble_mutex, portMAX_DELAY);
                    status_notify_enabled = (descr_value == 0x0001);
                    xSemaphoreGive(ble_mutex);
                    ESP_LOGI(TAG, "Status notifications %s", 
                             status_notify_enabled ? "enabled" : "disabled");
                }
                
                // Handle Progress CCC write
                else if (param->write.handle == attr_handle_table[IDX_PROGRESS_CFG] && 
                         param->write.len == 2) {
                    uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                    xSemaphoreTake(ble_mutex, portMAX_DELAY);
                    progress_notify_enabled = (descr_value == 0x0001);
                    xSemaphoreGive(ble_mutex);
                    ESP_LOGI(TAG, "Progress notifications %s", 
                             progress_notify_enabled ? "enabled" : "disabled");
                }
            }
            break;

        case ESP_GATTS_START_EVT:
            if (param->start.status == ESP_GATT_OK) {
                ESP_LOGD(TAG, "Service started");
            }
            break;

        case ESP_GATTS_READ_EVT:
            ESP_LOGD(TAG, "GATT read, handle=%d", param->read.handle);
            break;

        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(TAG, "MTU exchange: %d", param->mtu.mtu);
            break;

        default:
            break;
    }
}

// ============================================================================
// Public API
// ============================================================================

esp_err_t bluetooth_init(void) {
    esp_err_t ret;
    
    ble_mutex = xSemaphoreCreateMutex();
    if (ble_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_FAIL;
    }
    
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret) {
        ESP_LOGW(TAG, "BT classic mem release: %s", esp_err_to_name(ret));
    }
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
    
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "GAP register failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "GATTS register failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_ble_gatts_app_register(0);
    if (ret) {
        ESP_LOGE(TAG, "GATTS app register failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    esp_ble_gatt_set_local_mtu(500);
    
    ESP_LOGI(TAG, "✅ Bluetooth initialized");
    return ESP_OK;
}

esp_err_t bluetooth_start(void) {
    return esp_ble_gap_start_advertising(&adv_params);
}

esp_err_t bluetooth_stop(void) {
    return esp_ble_gap_stop_advertising();
}

esp_err_t bluetooth_send_status(lock_status_t status) {
    if (ble_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(ble_mutex, portMAX_DELAY);
    
    if (!ble_connected || !status_notify_enabled) {
        xSemaphoreGive(ble_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t data = (uint8_t)status;
    status_char_value = data;
    
    esp_err_t ret = esp_ble_gatts_send_indicate(ble_gatts_if, ble_conn_id, 
                                                 attr_handle_table[IDX_STATUS_VAL],
                                                 sizeof(data), &data, false);
    
    xSemaphoreGive(ble_mutex);
    
    return ret;
}

esp_err_t bluetooth_send_progress(uint8_t progress) {
    if (ble_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(ble_mutex, portMAX_DELAY);
    
    if (!ble_connected || !progress_notify_enabled) {
        xSemaphoreGive(ble_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    progress_char_value = progress;
    
    esp_err_t ret = esp_ble_gatts_send_indicate(ble_gatts_if, ble_conn_id, 
                                                 attr_handle_table[IDX_PROGRESS_VAL],
                                                 sizeof(progress), &progress, false);
    
    xSemaphoreGive(ble_mutex);
    
    return ret;
}

lock_command_t bluetooth_get_pending_command(void) {
    lock_command_t cmd;
    
    xSemaphoreTake(ble_mutex, portMAX_DELAY);
    cmd = pending_command;
    xSemaphoreGive(ble_mutex);
    
    return cmd;
}

void bluetooth_clear_command(void) {
    xSemaphoreTake(ble_mutex, portMAX_DELAY);
    pending_command = LOCK_CMD_NONE;
    xSemaphoreGive(ble_mutex);
}

bool bluetooth_is_connected(void) {
    if (ble_mutex == NULL) {
        return false;
    }
    
    xSemaphoreTake(ble_mutex, portMAX_DELAY);
    bool connected = ble_connected;
    xSemaphoreGive(ble_mutex);
    
    return connected;
}

void bluetooth_set_command_callback(bluetooth_cmd_callback_t callback) {
    cmd_callback = callback;
}
