#include "http.h"
#include "config.h"
#include "lock_state.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "HTTP";

// HTTP server handle
static httpd_handle_t server = NULL;

// Command handling
static lock_command_t pending_command = LOCK_CMD_NONE;
static http_cmd_callback_t cmd_callback = NULL;
static SemaphoreHandle_t http_mutex = NULL;

// Current lock status
static lock_status_t current_status = LOCK_STATUS_UNKNOWN;

// ============================================================================
// HTTP Handlers
// ============================================================================

// GET /status - Returns current lock status as JSON (detailed from lock_state)
static esp_err_t status_handler(httpd_req_t *req) {
    char *json_string = lock_state_get_json_status();
    
    if (json_string != NULL) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, json_string);
        free(json_string);
    } else {
        // Fallback to simple status
        cJSON *root = cJSON_CreateObject();
        const char *state_str = "unknown";
        switch (current_status) {
            case LOCK_STATUS_LOCKED:   state_str = "locked"; break;
            case LOCK_STATUS_UNLOCKED: state_str = "unlocked"; break;
            case LOCK_STATUS_MOVING:   state_str = "moving"; break;
            case LOCK_STATUS_ERROR:    state_str = "error"; break;
            default:                   state_str = "unknown"; break;
        }
        cJSON_AddStringToObject(root, "state", state_str);
        char *fallback_json = cJSON_Print(root);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, fallback_json);
        free(fallback_json);
        cJSON_Delete(root);
    }
    
    return ESP_OK;
}

// GET /lock - Lock the mechanism
static esp_err_t lock_handler(httpd_req_t *req) {
    xSemaphoreTake(http_mutex, portMAX_DELAY);
    pending_command = LOCK_CMD_LOCK;
    xSemaphoreGive(http_mutex);
    
    if (cmd_callback) {
        cmd_callback(LOCK_CMD_LOCK);
    }
    
    httpd_resp_sendstr(req, "{\"command\":\"lock\",\"status\":\"accepted\"}");
    return ESP_OK;
}

// GET /unlock - Unlock the mechanism
static esp_err_t unlock_handler(httpd_req_t *req) {
    xSemaphoreTake(http_mutex, portMAX_DELAY);
    pending_command = LOCK_CMD_UNLOCK;
    xSemaphoreGive(http_mutex);
    
    if (cmd_callback) {
        cmd_callback(LOCK_CMD_UNLOCK);
    }
    
    httpd_resp_sendstr(req, "{\"command\":\"unlock\",\"status\":\"accepted\"}");
    return ESP_OK;
}

// GET /calibrate - Run calibration
static esp_err_t calibrate_handler(httpd_req_t *req) {
    xSemaphoreTake(http_mutex, portMAX_DELAY);
    pending_command = LOCK_CMD_CALIBRATE;
    xSemaphoreGive(http_mutex);
    
    if (cmd_callback) {
        cmd_callback(LOCK_CMD_CALIBRATE);
    }
    
    httpd_resp_sendstr(req, "{\"command\":\"calibrate\",\"status\":\"accepted\"}");
    return ESP_OK;
}

// GET /stop - Emergency stop
static esp_err_t stop_handler(httpd_req_t *req) {
    xSemaphoreTake(http_mutex, portMAX_DELAY);
    pending_command = LOCK_CMD_STOP;
    xSemaphoreGive(http_mutex);
    
    if (cmd_callback) {
        cmd_callback(LOCK_CMD_STOP);
    }
    
    httpd_resp_sendstr(req, "{\"command\":\"stop\",\"status\":\"accepted\"}");
    return ESP_OK;
}

// ============================================================================
// HTTP Server API
// ============================================================================

esp_err_t http_init(void) {
    http_mutex = xSemaphoreCreateMutex();
    if (http_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "HTTP REST API initialized");
    return ESP_OK;
}

esp_err_t http_start(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 8;
    config.stack_size = 8192;
    
    ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);
    
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return ESP_FAIL;
    }
    
    // Register handlers
    httpd_uri_t status_uri = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = status_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &status_uri);
    
    httpd_uri_t lock_uri = {
        .uri = "/lock",
        .method = HTTP_GET,
        .handler = lock_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &lock_uri);
    
    httpd_uri_t unlock_uri = {
        .uri = "/unlock",
        .method = HTTP_GET,
        .handler = unlock_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &unlock_uri);
    
    httpd_uri_t calibrate_uri = {
        .uri = "/calibrate",
        .method = HTTP_GET,
        .handler = calibrate_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &calibrate_uri);
    
    httpd_uri_t stop_uri = {
        .uri = "/stop",
        .method = HTTP_GET,
        .handler = stop_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &stop_uri);
    
    ESP_LOGI(TAG, "✅ HTTP server started");
    ESP_LOGI(TAG, "   All endpoints are GET - just paste URL in browser!");
    return ESP_OK;
}

void http_stop(void) {
    if (server) {
        httpd_stop(server);
        server = NULL;
    }
}

esp_err_t http_set_status(lock_status_t status) {
    xSemaphoreTake(http_mutex, portMAX_DELAY);
    current_status = status;
    xSemaphoreGive(http_mutex);
    
    return ESP_OK;
}

lock_command_t http_get_pending_command(void) {
    lock_command_t cmd = LOCK_CMD_NONE;
    if (http_mutex) {
        xSemaphoreTake(http_mutex, portMAX_DELAY);
        cmd = pending_command;
        xSemaphoreGive(http_mutex);
    }
    return cmd;
}

void http_clear_command(void) {
    if (http_mutex) {
        xSemaphoreTake(http_mutex, portMAX_DELAY);
        pending_command = LOCK_CMD_NONE;
        xSemaphoreGive(http_mutex);
    }
}

bool http_is_running(void) {
    return server != NULL;
}

void http_set_command_callback(http_cmd_callback_t callback) {
    cmd_callback = callback;
}

