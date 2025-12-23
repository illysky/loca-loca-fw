#include "as5600.h"
#include "config.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "AS5600";

// ============================================================================
// Internal State
// ============================================================================

static struct {
    bool initialized;
    uint16_t last_raw_angle;     // Previous angle for wrap detection
    uint16_t start_raw_angle;    // Angle at reset (for calculating delta)
    int32_t revolutions;         // Complete revolutions count
    bool first_reading;          // Flag for first reading after reset
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
} encoder = {0};

// ============================================================================
// I2C Communication (New ESP-IDF 6.0+ API)
// ============================================================================

static esp_err_t i2c_init(void) {
    // Configure I2C master bus
    i2c_master_bus_config_t bus_config = {
        .i2c_port = AS5600_I2C_PORT,
        .sda_io_num = PIN_AS5600_SDA,
        .scl_io_num = PIN_AS5600_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = true,
    };
    
    esp_err_t ret = i2c_new_master_bus(&bus_config, &encoder.bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Add AS5600 device to the bus
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_I2C_ADDR,
        .scl_speed_hz = AS5600_I2C_FREQ_HZ,
    };
    
    ret = i2c_master_bus_add_device(encoder.bus_handle, &dev_config, &encoder.dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C add device failed: %s", esp_err_to_name(ret));
        i2c_del_master_bus(encoder.bus_handle);
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C initialized - SDA: GPIO%d, SCL: GPIO%d (internal pullups enabled)", 
             PIN_AS5600_SDA, PIN_AS5600_SCL);
    return ESP_OK;
}

static esp_err_t as5600_read_reg(uint8_t reg, uint8_t *data, size_t len) {
    // Write register address, then read data
    esp_err_t ret = i2c_master_transmit_receive(
        encoder.dev_handle,
        &reg,           // Write buffer (register address)
        1,              // Write length
        data,           // Read buffer
        len,            // Read length
        100             // Timeout in ms
    );
    
    return ret;
}

// ============================================================================
// Initialization
// ============================================================================

esp_err_t as5600_init(void) {
    ESP_LOGI(TAG, "Initializing AS5600 rotary encoder");
    
    // Initialize I2C
    esp_err_t ret = i2c_init();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Try to read status to verify communication
    uint8_t status;
    ret = as5600_read_reg(AS5600_REG_STATUS, &status, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to communicate with AS5600: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Check magnet status
    bool magnet_ok = (status & AS5600_STATUS_MD) != 0;
    bool magnet_strong = (status & AS5600_STATUS_MH) != 0;
    bool magnet_weak = (status & AS5600_STATUS_ML) != 0;
    
    ESP_LOGI(TAG, "AS5600 status: 0x%02X", status);
    ESP_LOGI(TAG, "  Magnet detected: %s", magnet_ok ? "YES" : "NO");
    if (magnet_strong) {
        ESP_LOGW(TAG, "  ⚠️  Magnet too STRONG - move magnet further away");
    }
    if (magnet_weak) {
        ESP_LOGW(TAG, "  ⚠️  Magnet too WEAK - move magnet closer");
    }
    
    // Read AGC for diagnostic
    uint8_t agc;
    ret = as5600_read_reg(AS5600_REG_AGC, &agc, 1);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "  AGC value: %d (ideal: 128, range: 0-255)", agc);
    }
    
    // Initialize tracking state
    encoder.initialized = true;
    encoder.revolutions = 0;
    encoder.first_reading = true;
    encoder.last_raw_angle = 0;
    
    // Do initial reading
    as5600_update();
    
    ESP_LOGI(TAG, "✅ AS5600 initialized successfully");
    return ESP_OK;
}

// ============================================================================
// Angle Reading Functions
// ============================================================================

esp_err_t as5600_read_raw_angle(uint16_t *raw_angle) {
    if (!encoder.initialized || raw_angle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t data[2];
    esp_err_t ret = as5600_read_reg(AS5600_REG_RAW_ANGLE_H, data, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    *raw_angle = ((uint16_t)data[0] << 8) | data[1];
    *raw_angle &= 0x0FFF;  // 12-bit value
    
    return ESP_OK;
}

esp_err_t as5600_read_angle(uint16_t *angle) {
    if (!encoder.initialized || angle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t data[2];
    esp_err_t ret = as5600_read_reg(AS5600_REG_ANGLE_H, data, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    *angle = ((uint16_t)data[0] << 8) | data[1];
    *angle &= 0x0FFF;  // 12-bit value
    
    return ESP_OK;
}

esp_err_t as5600_get_degrees(float *degrees) {
    uint16_t raw;
    esp_err_t ret = as5600_read_raw_angle(&raw);
    if (ret != ESP_OK) {
        return ret;
    }
    
    *degrees = (float)raw * 360.0f / 4096.0f;
    return ESP_OK;
}

// ============================================================================
// Status Functions
// ============================================================================

esp_err_t as5600_magnet_detected(bool *detected) {
    if (!encoder.initialized || detected == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t status;
    esp_err_t ret = as5600_read_reg(AS5600_REG_STATUS, &status, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    *detected = (status & AS5600_STATUS_MD) != 0;
    return ESP_OK;
}

esp_err_t as5600_get_agc(uint8_t *agc) {
    if (!encoder.initialized || agc == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    return as5600_read_reg(AS5600_REG_AGC, agc, 1);
}

esp_err_t as5600_get_magnitude(uint16_t *magnitude) {
    if (!encoder.initialized || magnitude == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t data[2];
    esp_err_t ret = as5600_read_reg(AS5600_REG_MAGNITUDE_H, data, 2);
    if (ret != ESP_OK) {
        return ret;
    }
    
    *magnitude = ((uint16_t)data[0] << 8) | data[1];
    *magnitude &= 0x0FFF;
    
    return ESP_OK;
}

// ============================================================================
// Revolution Tracking
// ============================================================================

void as5600_reset_revolutions(void) {
    encoder.revolutions = 0;
    encoder.first_reading = true;
    
    // Do immediate reading to set baseline
    uint16_t raw;
    if (as5600_read_raw_angle(&raw) == ESP_OK) {
        encoder.last_raw_angle = raw;
        encoder.start_raw_angle = raw;  // Remember starting position
        encoder.first_reading = false;
    }
    
    ESP_LOGI(TAG, "Revolution counter reset (start angle: %d)", encoder.start_raw_angle);
}

esp_err_t as5600_update(void) {
    uint16_t raw;
    esp_err_t ret = as5600_read_raw_angle(&raw);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (encoder.first_reading) {
        encoder.last_raw_angle = raw;
        encoder.first_reading = false;
        return ESP_OK;
    }
    
    // Detect wrap-around (revolution crossing)
    // If we go from ~4095 to ~0, that's a positive revolution (CW)
    // If we go from ~0 to ~4095, that's a negative revolution (CCW)
    int16_t delta = (int16_t)raw - (int16_t)encoder.last_raw_angle;
    
    // Threshold for detecting wrap (about 1/4 rotation = 1024 counts)
    const int16_t wrap_threshold = 2048;
    
    if (delta > wrap_threshold) {
        // Wrapped from low to high (CCW direction)
        encoder.revolutions--;
    } else if (delta < -wrap_threshold) {
        // Wrapped from high to low (CW direction)
        encoder.revolutions++;
    }
    
    encoder.last_raw_angle = raw;
    return ESP_OK;
}

esp_err_t as5600_get_state(as5600_state_t *state) {
    if (!encoder.initialized || state == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Update first to get latest reading
    esp_err_t ret = as5600_update();
    if (ret != ESP_OK) {
        return ret;
    }
    
    state->revolutions = encoder.revolutions;
    state->raw_angle = encoder.last_raw_angle;
    
    // Calculate total degrees including revolutions
    float angle_degrees = (float)encoder.last_raw_angle * 360.0f / 4096.0f;
    state->total_degrees = (float)encoder.revolutions * 360.0f + angle_degrees;
    
    // Get magnet status
    bool detected;
    if (as5600_magnet_detected(&detected) == ESP_OK) {
        state->magnet_detected = detected;
    } else {
        state->magnet_detected = false;
    }
    
    return ESP_OK;
}

float as5600_get_total_revolutions(void) {
    as5600_update();
    
    // Calculate delta from start position (handles wrap-around)
    int16_t delta = (int16_t)encoder.last_raw_angle - (int16_t)encoder.start_raw_angle;
    float angle_fraction = (float)delta / 4096.0f;
    
    return (float)encoder.revolutions + angle_fraction;
}

void as5600_print_status(void) {
    as5600_state_t state;
    if (as5600_get_state(&state) == ESP_OK) {
        float total_revs = (float)state.revolutions + 
                          ((float)state.raw_angle / 4096.0f);
        
        ESP_LOGI(TAG, "┌─────────────────────────────────────┐");
        ESP_LOGI(TAG, "│        AS5600 Encoder Status        │");
        ESP_LOGI(TAG, "├─────────────────────────────────────┤");
        ESP_LOGI(TAG, "│ Raw angle:     %4d / 4096          │", state.raw_angle);
        ESP_LOGI(TAG, "│ Angle:         %6.1f°              │", 
                 (float)state.raw_angle * 360.0f / 4096.0f);
        ESP_LOGI(TAG, "│ Revolutions:   %ld                   │", (long)state.revolutions);
        ESP_LOGI(TAG, "│ Total revs:    %.2f                │", total_revs);
        ESP_LOGI(TAG, "│ Total degrees: %.1f°              │", state.total_degrees);
        ESP_LOGI(TAG, "│ Magnet:        %s                  │", 
                 state.magnet_detected ? "OK" : "NOT DETECTED");
        ESP_LOGI(TAG, "└─────────────────────────────────────┘");
    } else {
        ESP_LOGE(TAG, "Failed to read AS5600 status");
    }
}
