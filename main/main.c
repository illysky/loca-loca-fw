#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mdns.h"

#include "config.h"
#include "wifi.h"
#include "http.h"
#include "stepper.h"
#include "tmc2209.h"
#include "as5600.h"
#include "lock_state.h"

static const char *TAG = "LOCK";

// ============================================================================
// LED Status Indication
// ============================================================================

static void led_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(PIN_LED, 0);
}

static void led_set(bool on) {
    gpio_set_level(PIN_LED, on ? 1 : 0);
}

static void led_blink(int count, int on_ms, int off_ms) {
    for (int i = 0; i < count; i++) {
        led_set(true);
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        led_set(false);
        if (i < count - 1) {
            vTaskDelay(pdMS_TO_TICKS(off_ms));
        }
    }
}

// ============================================================================
// Command Processing
// ============================================================================

static void process_command(lock_command_t cmd) {
    esp_err_t ret;
    
    switch (cmd) {
        case LOCK_CMD_LOCK:
            ESP_LOGI(TAG, "🔒 LOCK command received");
            http_set_status(LOCK_STATUS_MOVING);
            
            if (!lock_state_is_calibrated()) {
                ESP_LOGE(TAG, "❌ Not calibrated - run /calibrate first");
                http_set_status(LOCK_STATUS_ERROR);
                break;
            }
            
            ret = lock_state_move_to_lock();
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "✅ Lock engaged successfully");
                http_set_status(LOCK_STATUS_LOCKED);
            } else {
                // Lock failed - lever not down, returned to start
                ESP_LOGE(TAG, "❌ Lock failed - lever not down!");
                http_set_status(LOCK_STATUS_UNLOCKED);  // Status remains unlocked
            }
            break;
            
        case LOCK_CMD_UNLOCK:
            ESP_LOGI(TAG, "🔓 UNLOCK command received");
            http_set_status(LOCK_STATUS_MOVING);
            
            if (!lock_state_is_calibrated()) {
                ESP_LOGE(TAG, "❌ Not calibrated - run /calibrate first");
                http_set_status(LOCK_STATUS_ERROR);
                break;
            }
            
            ret = lock_state_move_to_unlock();
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "✅ Lock disengaged successfully");
                http_set_status(LOCK_STATUS_UNLOCKED);
            } else {
                ESP_LOGE(TAG, "❌ Lock disengage failed");
                http_set_status(LOCK_STATUS_ERROR);
            }
            break;
            
        case LOCK_CMD_CALIBRATE:
            ESP_LOGI(TAG, "🔧 CALIBRATE command received");
            http_set_status(LOCK_STATUS_CALIBRATING);
            
            ret = lock_state_calibrate();
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "✅ Calibration complete");
                lock_state_print_status();
                http_set_status(LOCK_STATUS_LOCKED);
            } else {
                ESP_LOGE(TAG, "❌ Calibration failed");
                http_set_status(LOCK_STATUS_ERROR);
            }
            break;
            
        case LOCK_CMD_STATUS:
            ESP_LOGI(TAG, "📊 STATUS request");
            lock_state_print_status();
            tmc2209_print_status();
            break;
            
        case LOCK_CMD_STOP:
            ESP_LOGW(TAG, "🛑 EMERGENCY STOP");
            stepper_stop();
            stepper_disable();
            http_set_status(LOCK_STATUS_ERROR);
            break;
            
        default:
            break;
    }
}

// ============================================================================
// Command Callback (called from BLE context)
// ============================================================================

static void http_command_callback(lock_command_t cmd) {
    // Just log here - actual processing happens in main task
    ESP_LOGI(TAG, "HTTP command callback: %d", cmd);
}

// ============================================================================
// Lock Control Task
// ============================================================================

static void lock_control_task(void *pvParameters) {
    ESP_LOGI(TAG, "Lock control task started");
    
    while (1) {
        // Check for pending HTTP commands
        lock_command_t cmd = http_get_pending_command();
        if (cmd != LOCK_CMD_NONE) {
            http_clear_command();
            process_command(cmd);
        }
        
        // Periodic status check (every 5 seconds)
        static uint32_t last_status_check = 0;
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        if (now - last_status_check > 5000) {
            last_status_check = now;
            
            // LED is dedicated to WiFi status (controlled by wifi.c)
            // Lock state is available via /status endpoint
            
            #if DEBUG_STEPPER_STATUS
            lock_state_print_status();
            #endif
            
            #if DEBUG_TMC_REGISTERS
            tmc2209_print_status();
            #endif
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ============================================================================
// Main Application Entry Point
// ============================================================================

void app_main(void) {
    ESP_LOGI(TAG, "╔════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║     ESP32-C3 Smart Lock Controller     ║");
    ESP_LOGI(TAG, "║     TMC2209 + StallGuard Homing        ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════╝");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS flash erase required");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize LED
    led_init();
    led_blink(3, 100, 100);  // Startup indication
    
    // Print GPIO configuration
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== GPIO Configuration ===");
    ESP_LOGI(TAG, "  TMC2209 STEP:  GPIO%d", PIN_TMC_STEP);
    ESP_LOGI(TAG, "  TMC2209 DIR:   GPIO%d", PIN_TMC_DIR);
    ESP_LOGI(TAG, "  TMC2209 EN:    GPIO%d (active LOW)", PIN_TMC_EN);
    ESP_LOGI(TAG, "  TMC2209 DIAG:  GPIO%d (StallGuard)", PIN_TMC_DIAG);
    ESP_LOGI(TAG, "  TMC2209 INDEX: GPIO%d", PIN_TMC_INDEX);
    ESP_LOGI(TAG, "  TMC2209 MS1:   GPIO%d", PIN_TMC_MS1);
    ESP_LOGI(TAG, "  TMC2209 MS2:   GPIO%d", PIN_TMC_MS2);
    ESP_LOGI(TAG, "  TMC2209 UART:  GPIO%d (single-wire)", PIN_TMC_UART_TX);
    ESP_LOGI(TAG, "  AS5600 SDA:    GPIO%d (I2C)", PIN_AS5600_SDA);
    ESP_LOGI(TAG, "  AS5600 SCL:    GPIO%d (I2C)", PIN_AS5600_SCL);
    ESP_LOGI(TAG, "  Status LED:    GPIO%d", PIN_LED);
    ESP_LOGI(TAG, "");
    
    // Print motor configuration
    ESP_LOGI(TAG, "=== Motor Configuration ===");
    ESP_LOGI(TAG, "  Run current:   %d mA", TMC_RUN_CURRENT_MA);
    ESP_LOGI(TAG, "  Hold current:  %d mA", TMC_HOLD_CURRENT_MA);
    ESP_LOGI(TAG, "  Microsteps:    %d", TMC_MICROSTEPS);
    ESP_LOGI(TAG, "  SG threshold:  %d", TMC_STALLGUARD_THRESH);
    ESP_LOGI(TAG, "  StealthChop:   %s", TMC_STEALTHCHOP ? "yes" : "no");
    ESP_LOGI(TAG, "  Max speed:     %d Hz", MOTOR_MAX_SPEED_HZ);
    ESP_LOGI(TAG, "  Max steps:     %d", LOCK_MAX_STEPS);
    ESP_LOGI(TAG, "");
    
    // Initialize WiFi
    ESP_LOGI(TAG, "Initializing WiFi...");
    ret = wifi_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed: %s", esp_err_to_name(ret));
        led_blink(10, 100, 100);
        return;
    }
    
    ESP_LOGI(TAG, "Connecting to WiFi: %s", WIFI_SSID);
    ret = wifi_connect(WIFI_SSID, WIFI_PASSWORD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi connection failed: %s", esp_err_to_name(ret));
        led_blink(10, 100, 100);
        return;
    }
    
    char ip_str[16];
    wifi_get_ip(ip_str);
    ESP_LOGI(TAG, "✅ WiFi connected! IP: %s", ip_str);
    
    // Initialize mDNS
    ESP_LOGI(TAG, "Initializing mDNS...");
    ret = mdns_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mDNS init failed: %s", esp_err_to_name(ret));
    } else {
        mdns_hostname_set("shack_lock");
        mdns_instance_name_set("Shack Lock Controller");
        mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
        ESP_LOGI(TAG, "✅ mDNS started: http://shack_lock.local");
    }
    
    // Initialize HTTP server
    ESP_LOGI(TAG, "Initializing HTTP server...");
    ret = http_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HTTP init failed: %s", esp_err_to_name(ret));
        led_blink(10, 100, 100);
        return;
    }
    
    http_set_command_callback(http_command_callback);
    ret = http_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HTTP start failed: %s", esp_err_to_name(ret));
        led_blink(10, 100, 100);
        return;
    }
    ESP_LOGI(TAG, "✅ HTTP server started on http://%s", ip_str);
    
    // Initialize AS5600 rotary encoder
    ESP_LOGI(TAG, "Initializing AS5600 rotary encoder...");
    ret = as5600_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "AS5600 init failed: %s (encoder tracking disabled)", esp_err_to_name(ret));
        // Don't return - encoder is optional, continue without it
    } else {
        ESP_LOGI(TAG, "✅ AS5600 encoder initialized");
        as5600_print_status();
    }
    
    // Initialize stepper motor system
    ESP_LOGI(TAG, "Initializing stepper motor...");
    ret = stepper_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Stepper init failed: %s", esp_err_to_name(ret));
        led_blink(10, 100, 100);  // Error indication
        return;
    }
    ESP_LOGI(TAG, "✅ Stepper motor initialized");
    
    // Initialize lock state tracking
    ESP_LOGI(TAG, "Initializing lock state tracking...");
    ret = lock_state_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Lock state init failed: %s", esp_err_to_name(ret));
        led_blink(10, 100, 100);
        return;
    }
    ESP_LOGI(TAG, "✅ Lock state tracking initialized");
    
    // Print wiring diagram
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== Wiring Diagram (Adafruit TMC2209) ===");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "  ESP32-C3          TMC2209 Breakout");
    ESP_LOGI(TAG, "  ────────          ────────────────");
    ESP_LOGI(TAG, "  3.3V    ───────►  VIO");
    ESP_LOGI(TAG, "  GND     ───────►  GND");
    ESP_LOGI(TAG, "  GPIO%d   ───────►  STEP", PIN_TMC_STEP);
    ESP_LOGI(TAG, "  GPIO%d   ───────►  DIR", PIN_TMC_DIR);
    ESP_LOGI(TAG, "  GPIO%d   ───────►  EN", PIN_TMC_EN);
    ESP_LOGI(TAG, "  GPIO%d   ───────►  MS1", PIN_TMC_MS1);
    ESP_LOGI(TAG, "  GPIO%d   ───────►  MS2", PIN_TMC_MS2);
    ESP_LOGI(TAG, "  GPIO%d  ◄───────  DIAG", PIN_TMC_DIAG);
    ESP_LOGI(TAG, "  GPIO%d  ◄───────  INDEX", PIN_TMC_INDEX);
    ESP_LOGI(TAG, "  GPIO%d  ──[1k]─►  PDN_UART (single-wire)", PIN_TMC_UART_TX);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "  External power:  VM = Motor voltage (8-28V)");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "  ESP32-C3          AS5600 Encoder");
    ESP_LOGI(TAG, "  ────────          ──────────────");
    ESP_LOGI(TAG, "  3.3V    ───────►  VCC");
    ESP_LOGI(TAG, "  GND     ───────►  GND");
    ESP_LOGI(TAG, "  GPIO%d   ◄──────►  SDA", PIN_AS5600_SDA);
    ESP_LOGI(TAG, "  GPIO%d   ───────►  SCL", PIN_AS5600_SCL);
    ESP_LOGI(TAG, "");
    
    // Print HTTP API info
    ESP_LOGI(TAG, "=== HTTP REST API (all GET - use in browser!) ===");
    ESP_LOGI(TAG, "  http://shack_lock.local/status     - Get detailed lock status (JSON)");
    ESP_LOGI(TAG, "  http://shack_lock.local/lock       - Lock the shack");
    ESP_LOGI(TAG, "  http://shack_lock.local/unlock     - Unlock the shack");
    ESP_LOGI(TAG, "  http://shack_lock.local/calibrate  - Run calibration (REQUIRED first!)");
    ESP_LOGI(TAG, "  http://shack_lock.local/stop       - Emergency stop");
    ESP_LOGI(TAG, "  (or use IP: http://%s)", ip_str);
    ESP_LOGI(TAG, "");
    
    // Create control task
    xTaskCreate(lock_control_task, "lock_ctrl", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "🚀 Lock controller started!");
    ESP_LOGI(TAG, "   ⚠️  Run /calibrate before first use!");
    
    // Main loop
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
