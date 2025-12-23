#include "wifi.h"
#include "config.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <string.h>

static const char *TAG = "WIFI";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static bool s_was_connected = false;  // Track if we were ever connected
static TaskHandle_t s_led_task = NULL;

// Helper to set LED with active-low support
static inline void wifi_led_set(bool on) {
    gpio_set_level(PIN_LED, (on ^ LED_ACTIVE_LOW) ? 1 : 0);
}

// LED blinking task for connection status
static void wifi_led_blink_task(void *pvParameters) {
    while (1) {
        wifi_led_set(true);
        vTaskDelay(pdMS_TO_TICKS(100));
        wifi_led_set(false);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void start_led_blink(void) {
    if (s_led_task == NULL) {
        xTaskCreate(wifi_led_blink_task, "wifi_led", 1024, NULL, 1, &s_led_task);
    }
}

static void stop_led_blink(bool led_on) {
    if (s_led_task != NULL) {
        vTaskDelete(s_led_task);
        s_led_task = NULL;
    }
    wifi_led_set(led_on);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi starting, attempting connection...");
        start_led_blink();  // Start blinking while connecting
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // Clear connected bit
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        start_led_blink();  // Blink while reconnecting
        
        s_retry_num++;
        
        if (s_was_connected) {
            // We were connected before - keep trying forever with backoff
            int delay_ms = (s_retry_num < 10) ? 1000 : 5000;  // 1s for first 10, then 5s
            ESP_LOGW(TAG, "WiFi disconnected! Reconnecting in %dms (attempt %d)...", 
                     delay_ms, s_retry_num);
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
            esp_wifi_connect();
        } else {
            // Initial connection - limited retries
            if (s_retry_num < 10) {
                ESP_LOGI(TAG, "Retrying WiFi connection (%d/10)...", s_retry_num);
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_wifi_connect();
            } else {
                stop_led_blink(false);  // LED off on failure
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                ESP_LOGE(TAG, "WiFi connection failed after %d attempts", s_retry_num);
            }
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "✅ WiFi connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        s_was_connected = true;  // Mark that we achieved connection
        stop_led_blink(true);  // LED solid on when connected
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t wifi_init(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    // Don't start WiFi yet - wait for wifi_connect() to set config first

    ESP_LOGI(TAG, "WiFi initialized");
    return ESP_OK;
}

esp_err_t wifi_connect(const char *ssid, const char *password) {
    if (ssid == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    if (password) {
        strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    }
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_LOGI(TAG, "Connecting to WiFi: %s", ssid);
    
    // Set config BEFORE starting WiFi
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    // Now start WiFi - this will trigger WIFI_EVENT_STA_START -> esp_wifi_connect()
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE,
                                            pdFALSE,
                                            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "✅ Connected to WiFi");
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "❌ Failed to connect to WiFi");
        return ESP_FAIL;
    } else {
        return ESP_ERR_TIMEOUT;
    }
}

bool wifi_is_connected(void) {
    return (xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT) != 0;
}

esp_err_t wifi_get_ip(char *ip_str) {
    if (ip_str == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK) {
        return ESP_FAIL;
    }

    sprintf(ip_str, IPSTR, IP2STR(&ip_info.ip));
    return ESP_OK;
}

