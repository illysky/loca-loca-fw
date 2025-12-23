#include "tmc2209.h"
#include "config.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "TMC2209";

// ============================================================================
// Internal Constants
// ============================================================================

#define TMC_UART_BUF_SIZE       256
#define TMC_SYNC_BYTE           0x05
#define TMC_WRITE_BIT           0x80

// Rsense value for Adafruit TMC2209 breakout (110mOhm)
#define TMC_RSENSE_MOHM         110

// ============================================================================
// Internal State
// ============================================================================

static bool tmc_initialized = false;
static uint8_t tmc_slave_addr = TMC_SLAVE_ADDR;

// Cached register values
static uint32_t cached_chopconf = 0;
static uint32_t cached_gconf = 0;
static uint32_t cached_ihold_irun = 0;

// ============================================================================
// CRC Calculation (TMC2209 uses CRC8)
// ============================================================================

static uint8_t calc_crc(uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (byte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = crc << 1;
            }
            byte >>= 1;
        }
    }
    return crc;
}

// ============================================================================
// UART Communication
// ============================================================================

static esp_err_t tmc_uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = TMC_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    esp_err_t ret = uart_driver_install(TMC_UART_NUM, TMC_UART_BUF_SIZE, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = uart_param_config(TMC_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Two-wire half-duplex: TX through resistor, RX direct - both to same TMC UART pin
    ret = uart_set_pin(TMC_UART_NUM, PIN_TMC_UART_TX, PIN_TMC_UART_RX, 
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Standard UART mode (separate TX/RX pins)
    uart_set_mode(TMC_UART_NUM, UART_MODE_UART);
    
    ESP_LOGI(TAG, "UART initialized TX:GPIO%d RX:GPIO%d @ %d baud", 
             PIN_TMC_UART_TX, PIN_TMC_UART_RX, TMC_UART_BAUD);
    
    return ESP_OK;
}

// Set microstep resolution via MS1/MS2 pins
// MS1  MS2  Result
//  0    0   8 microsteps
//  1    0   32 microsteps (interpolated to 256)
//  0    1   64 microsteps (interpolated to 256)
//  1    1   16 microsteps (interpolated to 256)
void tmc2209_set_ms_pins(uint16_t microsteps) {
    switch (microsteps) {
        case 8:
            gpio_set_level(PIN_TMC_MS1, 0);
            gpio_set_level(PIN_TMC_MS2, 0);
            break;
        case 32:
            gpio_set_level(PIN_TMC_MS1, 1);
            gpio_set_level(PIN_TMC_MS2, 0);
            break;
        case 64:
            gpio_set_level(PIN_TMC_MS1, 0);
            gpio_set_level(PIN_TMC_MS2, 1);
            break;
        case 16:
        default:
            gpio_set_level(PIN_TMC_MS1, 1);
            gpio_set_level(PIN_TMC_MS2, 1);
            break;
    }
    ESP_LOGI(TAG, "MS pins set for %d microsteps", microsteps);
}

static esp_err_t tmc_gpio_init(void) {
    // Configure STEP pin
    gpio_config_t step_conf = {
        .pin_bit_mask = (1ULL << PIN_TMC_STEP),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&step_conf);
    gpio_set_level(PIN_TMC_STEP, 0);
    
    // Configure DIR pin
    gpio_config_t dir_conf = {
        .pin_bit_mask = (1ULL << PIN_TMC_DIR),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&dir_conf);
    gpio_set_level(PIN_TMC_DIR, 0);
    
    // Configure EN pin (active LOW - start disabled)
    gpio_config_t en_conf = {
        .pin_bit_mask = (1ULL << PIN_TMC_EN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&en_conf);
    gpio_set_level(PIN_TMC_EN, 1);  // Disabled initially
    
    // Configure DIAG pin (input for StallGuard)
    gpio_config_t diag_conf = {
        .pin_bit_mask = (1ULL << PIN_TMC_DIAG),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&diag_conf);
    
    // Configure INDEX pin (input from TMC2209)
    gpio_config_t index_conf = {
        .pin_bit_mask = (1ULL << PIN_TMC_INDEX),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&index_conf);
    
    // Configure MS1 pin (microstep select)
    gpio_config_t ms1_conf = {
        .pin_bit_mask = (1ULL << PIN_TMC_MS1),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&ms1_conf);
    
    // Configure MS2 pin (microstep select)
    gpio_config_t ms2_conf = {
        .pin_bit_mask = (1ULL << PIN_TMC_MS2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&ms2_conf);
    
    // Set default microstep mode (MS1=H, MS2=H = 16 microsteps with interpolation)
    gpio_set_level(PIN_TMC_MS1, 1);
    gpio_set_level(PIN_TMC_MS2, 1);
    
    ESP_LOGI(TAG, "GPIO initialized:");
    ESP_LOGI(TAG, "  STEP:%d DIR:%d EN:%d", PIN_TMC_STEP, PIN_TMC_DIR, PIN_TMC_EN);
    ESP_LOGI(TAG, "  DIAG:%d INDEX:%d", PIN_TMC_DIAG, PIN_TMC_INDEX);
    ESP_LOGI(TAG, "  MS1:%d MS2:%d", PIN_TMC_MS1, PIN_TMC_MS2);
    
    return ESP_OK;
}

// ============================================================================
// Register Access
// ============================================================================

esp_err_t tmc2209_write_reg(uint8_t reg, uint32_t value) {
    if (!tmc_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t tx_buf[8];
    
    // Build write datagram
    tx_buf[0] = TMC_SYNC_BYTE;
    tx_buf[1] = tmc_slave_addr;
    tx_buf[2] = reg | TMC_WRITE_BIT;
    tx_buf[3] = (value >> 24) & 0xFF;
    tx_buf[4] = (value >> 16) & 0xFF;
    tx_buf[5] = (value >> 8) & 0xFF;
    tx_buf[6] = value & 0xFF;
    tx_buf[7] = calc_crc(tx_buf, 7);
    
    // Clear RX buffer (in case of echoed data on single-wire setup)
    uart_flush_input(TMC_UART_NUM);
    
    // Send data
    int written = uart_write_bytes(TMC_UART_NUM, (const char *)tx_buf, 8);
    if (written != 8) {
        ESP_LOGE(TAG, "Write failed, sent %d of 8 bytes", written);
        return ESP_FAIL;
    }
    
    // Wait for transmission to complete
    uart_wait_tx_done(TMC_UART_NUM, pdMS_TO_TICKS(10));
    
    // Small delay for TMC to process
    vTaskDelay(pdMS_TO_TICKS(2));
    
    // Discard echo (single-wire UART reflects our TX)
    uart_flush_input(TMC_UART_NUM);
    
    ESP_LOGD(TAG, "Write reg 0x%02X = 0x%08lX", reg, (unsigned long)value);
    
    return ESP_OK;
}

esp_err_t tmc2209_read_reg(uint8_t reg, uint32_t *value) {
    if (!tmc_initialized || value == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t tx_buf[4];
    uint8_t rx_buf[12];  // Extra space for echo + response
    
    // Build read request datagram
    tx_buf[0] = TMC_SYNC_BYTE;
    tx_buf[1] = tmc_slave_addr;
    tx_buf[2] = reg & 0x7F;  // Read: bit 7 = 0
    tx_buf[3] = calc_crc(tx_buf, 3);
    
    // Clear RX buffer
    uart_flush_input(TMC_UART_NUM);
    
    // Clear RX buffer before sending
    uart_flush_input(TMC_UART_NUM);
    
    // Send read request
    int written = uart_write_bytes(TMC_UART_NUM, (const char *)tx_buf, 4);
    if (written != 4) {
        ESP_LOGE(TAG, "Read request failed");
        return ESP_FAIL;
    }
    
    // Wait for transmission to complete
    uart_wait_tx_done(TMC_UART_NUM, pdMS_TO_TICKS(10));
    
    // Wait for TMC2209 to process and respond
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Read response: 4 bytes echo + 8 bytes response = 12 bytes
    // Use longer timeout to ensure we get the response
    int len = uart_read_bytes(TMC_UART_NUM, rx_buf, 12, pdMS_TO_TICKS(100));
    
    // Debug: print received bytes
    if (len > 0) {
        char hex_buf[64];
        char *p = hex_buf;
        for (int i = 0; i < len && i < 20; i++) {
            p += sprintf(p, "%02X ", rx_buf[i]);
        }
        ESP_LOGI(TAG, "RX [%d bytes]: %s", len, hex_buf);
    }
    
    // Response format: sync(1) + addr(1) + reg(1) + data(4) + crc(1) = 8 bytes
    // With echo: 4 + 8 = 12 bytes
    
    if (len >= 8) {
        // Find response start (skip echo if present)
        int resp_start = (len >= 12) ? 4 : 0;
        
        // Verify response
        if (rx_buf[resp_start] == TMC_SYNC_BYTE && 
            rx_buf[resp_start + 1] == 0xFF &&    // Master addr
            rx_buf[resp_start + 2] == reg) {
            
            // Extract 32-bit value
            *value = ((uint32_t)rx_buf[resp_start + 3] << 24) |
                     ((uint32_t)rx_buf[resp_start + 4] << 16) |
                     ((uint32_t)rx_buf[resp_start + 5] << 8) |
                     ((uint32_t)rx_buf[resp_start + 6]);
            
            // Verify CRC
            uint8_t crc = calc_crc(&rx_buf[resp_start], 7);
            if (crc == rx_buf[resp_start + 7]) {
                ESP_LOGD(TAG, "Read reg 0x%02X = 0x%08lX", reg, (unsigned long)*value);
                return ESP_OK;
            } else {
                ESP_LOGW(TAG, "CRC mismatch on read");
            }
        }
    }
    
    ESP_LOGW(TAG, "Read failed for reg 0x%02X (got %d bytes)", reg, len);
    return ESP_FAIL;
}

// ============================================================================
// Initialization
// ============================================================================

esp_err_t tmc2209_init(void) {
    esp_err_t ret;
    
    // Initialize GPIO pins
    ret = tmc_gpio_init();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Initialize UART
    ret = tmc_uart_init();
    if (ret != ESP_OK) {
        return ret;
    }
    
    tmc_initialized = true;
    
    // Small delay for TMC to power up
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Try to read IOIN register to verify communication
    uint32_t ioin;
    ret = tmc2209_read_reg(TMC_REG_IOIN, &ioin);
    if (ret == ESP_OK) {
        uint8_t version = (ioin >> 24) & 0xFF;
        ESP_LOGI(TAG, "TMC2209 detected, version: 0x%02X", version);
    } else {
        ESP_LOGW(TAG, "Could not read TMC2209 (UART may need single-wire setup)");
        // Continue anyway - GPIO control still works
    }
    
    // Clear any errors
    tmc2209_write_reg(TMC_REG_GSTAT, 0x07);
    
    ESP_LOGI(TAG, "TMC2209 initialized");
    return ESP_OK;
}

// ============================================================================
// Configuration
// ============================================================================

// Calculate current scale from mA
static uint8_t current_to_scale(uint16_t current_ma) {
    // Formula: I_rms = (CS + 1) / 32 * V_fs / R_sense * 1/sqrt(2)
    // V_fs = 0.325V for TMC2209
    // Solving for CS: CS = (32 * I_rms * R_sense * sqrt(2) / 0.325) - 1
    
    float cs = (32.0f * current_ma * TMC_RSENSE_MOHM * 1.414f / 325000.0f) - 1.0f;
    
    if (cs < 0) cs = 0;
    if (cs > 31) cs = 31;
    
    return (uint8_t)cs;
}

static uint8_t microsteps_to_mres(uint16_t microsteps) {
    switch (microsteps) {
        case 256: return 0;
        case 128: return 1;
        case 64:  return 2;
        case 32:  return 3;
        case 16:  return 4;
        case 8:   return 5;
        case 4:   return 6;
        case 2:   return 7;
        case 1:   return 8;
        default:  return 4;  // 16 microsteps default
    }
}

esp_err_t tmc2209_configure(const tmc2209_config_t *config) {
    if (!tmc_initialized || config == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Configuring TMC2209:");
    ESP_LOGI(TAG, "  Run current: %d mA", config->run_current_ma);
    ESP_LOGI(TAG, "  Hold current: %d mA", config->hold_current_ma);
    ESP_LOGI(TAG, "  Microsteps: %d", config->microsteps);
    ESP_LOGI(TAG, "  StallGuard threshold: %d", config->stallguard_thresh);
    ESP_LOGI(TAG, "  StealthChop: %s", config->stealthchop ? "enabled" : "disabled");
    
    // Configure GCONF
    // Bit 0: I_scale_analog (0 = internal)
    // Bit 1: internal_Rsense (0 = external)
    // Bit 2: en_spreadcycle (0 = StealthChop, 1 = SpreadCycle)
    // Bit 3: shaft (direction)
    // Bit 4: index_otpw
    // Bit 5: index_step
    // Bit 6: pdn_disable (1 = UART control)
    // Bit 7: mstep_reg_select (1 = MRES via UART)
    // Bit 8: multistep_filt
    // Bit 9: test_mode
    cached_gconf = 0;
    cached_gconf |= (1 << 6);  // Enable UART (disable PDN_UART)
    cached_gconf |= (1 << 7);  // MRES via register
    cached_gconf |= (1 << 8);  // Multistep filter
    if (!config->stealthchop) {
        cached_gconf |= (1 << 2);  // Enable SpreadCycle
    }
    tmc2209_write_reg(TMC_REG_GCONF, cached_gconf);
    
    // Configure CHOPCONF
    // MRES: bits 24-27
    // Vsense: bit 17 (0 = full scale, 1 = reduced)
    // TBL: bits 15-16 (blank time)
    // TOFF: bits 0-3 (off time)
    // HSTRT: bits 4-6
    // HEND: bits 7-10
    cached_chopconf = 0;
    cached_chopconf |= microsteps_to_mres(config->microsteps) << 24;  // MRES
    cached_chopconf |= 2 << 15;     // TBL = 2 (blank_time(24) equivalent)
    cached_chopconf |= 4;           // TOFF = 4 (from working example, was 5)
    cached_chopconf |= 4 << 4;      // HSTRT = 4
    cached_chopconf |= 1 << 7;      // HEND = 1
    tmc2209_write_reg(TMC_REG_CHOPCONF, cached_chopconf);
    
    // Configure IHOLD_IRUN
    uint8_t irun = current_to_scale(config->run_current_ma);
    uint8_t ihold = current_to_scale(config->hold_current_ma);
    cached_ihold_irun = 0;
    cached_ihold_irun |= ihold;         // IHOLD (bits 0-4)
    cached_ihold_irun |= irun << 8;     // IRUN (bits 8-12)
    cached_ihold_irun |= 5 << 16;       // IHOLDDELAY (bits 16-19)
    tmc2209_write_reg(TMC_REG_IHOLD_IRUN, cached_ihold_irun);
    
    // Configure TPOWERDOWN (time until current reduction)
    tmc2209_write_reg(TMC_REG_TPOWERDOWN, 10);  // ~160ms
    
    // Configure TPWMTHRS (StealthChop threshold)
    // TSTEP < TPWMTHRS: StealthChop, TSTEP > TPWMTHRS: SpreadCycle
    tmc2209_write_reg(TMC_REG_TPWMTHRS, 0);  // Always StealthChop (when enabled)
    
    // DISABLE StallGuard - we use encoder-based stall detection instead
    // Set SGTHRS=0 and TCOOLTHRS=0 to completely disable StallGuard/CoolStep
    ESP_LOGI(TAG, "  StallGuard DISABLED (using encoder-based stall detection)");
    tmc2209_write_reg(TMC_REG_SGTHRS, 0);      // No stall threshold
    tmc2209_write_reg(TMC_REG_TCOOLTHRS, 0);   // Disable CoolStep/StallGuard
    
    // DISABLE CoolStep (not needed without StallGuard)
    ESP_LOGI(TAG, "  CoolStep DISABLED");
    tmc2209_write_reg(TMC_REG_COOLCONF, 0);
    
    // Configure PWMCONF for StealthChop
    uint32_t pwmconf = 0;
    pwmconf |= 0xC8;          // PWM_OFS
    pwmconf |= 0xFF << 8;     // PWM_GRAD
    pwmconf |= 1 << 18;       // PWM_AUTOSCALE
    pwmconf |= 1 << 19;       // PWM_AUTOGRAD
    pwmconf |= 1 << 16;       // PWM_FREQ = 1 (2/1024 fCLK)
    tmc2209_write_reg(TMC_REG_PWMCONF, pwmconf);
    
    // Readback verification
    ESP_LOGI(TAG, "=== Verifying configuration (readback) ===");
    
    uint32_t readback;
    
    // IFCNT - counts successful writes (should be non-zero after our writes)
    if (tmc2209_read_reg(TMC_REG_IFCNT, &readback) == ESP_OK) {
        ESP_LOGI(TAG, "  IFCNT: %lu successful UART writes received", (unsigned long)readback);
        if (readback == 0) {
            ESP_LOGE(TAG, "  ERROR: No writes received! Check UART TX wiring!");
        }
    }
    
    // GCONF - check SpreadCycle bit
    if (tmc2209_read_reg(TMC_REG_GCONF, &readback) == ESP_OK) {
        bool spreadcycle = (readback >> 2) & 1;
        ESP_LOGI(TAG, "  GCONF: 0x%08lX (SpreadCycle=%s)", 
                 (unsigned long)readback, spreadcycle ? "ON" : "OFF");
        if (spreadcycle != !config->stealthchop) {
            ESP_LOGE(TAG, "  ERROR: SpreadCycle mismatch! StallGuard won't work!");
        }
    } else {
        ESP_LOGE(TAG, "  GCONF readback FAILED");
    }
    
    // CHOPCONF - check MRES
    if (tmc2209_read_reg(TMC_REG_CHOPCONF, &readback) == ESP_OK) {
        uint8_t mres = (readback >> 24) & 0x0F;
        uint8_t toff = readback & 0x0F;
        ESP_LOGI(TAG, "  CHOPCONF: 0x%08lX (MRES=%d, TOFF=%d)", 
                 (unsigned long)readback, mres, toff);
        if (toff == 0) {
            ESP_LOGE(TAG, "  ERROR: TOFF=0 means driver disabled!");
        }
    } else {
        ESP_LOGE(TAG, "  CHOPCONF readback FAILED");
    }
    
    // NOTE: IHOLD_IRUN, SGTHRS, TCOOLTHRS are WRITE-ONLY registers!
    // They always read as 0. We rely on IFCNT to verify writes succeeded.
    ESP_LOGI(TAG, "  (IHOLD_IRUN, SGTHRS, TCOOLTHRS are write-only - cannot verify)");
    
    // DRV_STATUS - get initial SG_RESULT
    if (tmc2209_read_reg(TMC_REG_DRVSTATUS, &readback) == ESP_OK) {
        uint16_t sg_result = readback & 0x3FF;
        bool standstill = (readback >> 31) & 1;
        ESP_LOGI(TAG, "  DRV_STATUS: SG_RESULT=%d, Standstill=%s", 
                 sg_result, standstill ? "YES" : "NO");
    }
    
    ESP_LOGI(TAG, "=== Configuration complete ===");
    
    return ESP_OK;
}

// ============================================================================
// Current Control
// ============================================================================

esp_err_t tmc2209_set_run_current(uint16_t current_ma) {
    uint8_t irun = current_to_scale(current_ma);
    cached_ihold_irun = (cached_ihold_irun & ~(0x1F << 8)) | (irun << 8);
    return tmc2209_write_reg(TMC_REG_IHOLD_IRUN, cached_ihold_irun);
}

esp_err_t tmc2209_set_hold_current(uint16_t current_ma) {
    uint8_t ihold = current_to_scale(current_ma);
    cached_ihold_irun = (cached_ihold_irun & ~0x1F) | ihold;
    return tmc2209_write_reg(TMC_REG_IHOLD_IRUN, cached_ihold_irun);
}

// ============================================================================
// Microstepping
// ============================================================================

esp_err_t tmc2209_set_microsteps(uint16_t microsteps) {
    uint8_t mres = microsteps_to_mres(microsteps);
    cached_chopconf = (cached_chopconf & ~(0x0F << 24)) | (mres << 24);
    return tmc2209_write_reg(TMC_REG_CHOPCONF, cached_chopconf);
}

// ============================================================================
// StallGuard
// ============================================================================

esp_err_t tmc2209_set_stallguard_threshold(uint8_t threshold) {
    return tmc2209_write_reg(TMC_REG_SGTHRS, threshold);
}

bool tmc2209_stall_detected(void) {
    // Check DIAG pin (active HIGH when stall detected)
    return gpio_get_level(PIN_TMC_DIAG) == 1;
}

uint16_t tmc2209_get_sg_result(void) {
    uint32_t result;
    if (tmc2209_read_reg(TMC_REG_SG_RESULT, &result) == ESP_OK) {
        return result & 0x3FF;  // 10-bit value
    }
    return 0xFFFF;  // Error indicator
}

// ============================================================================
// Mode Control
// ============================================================================

esp_err_t tmc2209_set_stealthchop(bool enable) {
    if (enable) {
        cached_gconf &= ~(1 << 2);  // Disable SpreadCycle = enable StealthChop
    } else {
        cached_gconf |= (1 << 2);   // Enable SpreadCycle
    }
    return tmc2209_write_reg(TMC_REG_GCONF, cached_gconf);
}

// ============================================================================
// Enable/Disable
// ============================================================================

void tmc2209_enable(void) {
    gpio_set_level(PIN_TMC_EN, 0);  // Active LOW
    ESP_LOGI(TAG, "Driver enabled");
}

void tmc2209_disable(void) {
    gpio_set_level(PIN_TMC_EN, 1);  // Inactive HIGH
    ESP_LOGI(TAG, "Driver disabled");
}

void tmc2209_reset(void) {
    ESP_LOGI(TAG, "Resetting TMC2209...");
    
    // Disable driver
    gpio_set_level(PIN_TMC_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Clear all error flags
    tmc2209_write_reg(TMC_REG_GSTAT, 0x07);
    
    // Reset SGTHRS to 0
    tmc2209_write_reg(TMC_REG_SGTHRS, 0);
    
    // Wait for TMC to settle
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Keep driver DISABLED - motor can spin free
    // Driver will be enabled when movement starts
    
    ESP_LOGI(TAG, "TMC2209 reset complete, DIAG=%d, driver DISABLED (motor free)", 
             gpio_get_level(PIN_TMC_DIAG));
}

// ============================================================================
// Status
// ============================================================================

esp_err_t tmc2209_get_status(tmc2209_status_t *status) {
    if (!tmc_initialized || status == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint32_t drv_status;
    esp_err_t ret = tmc2209_read_reg(TMC_REG_DRVSTATUS, &drv_status);
    if (ret != ESP_OK) {
        return ret;
    }
    
    status->standstill = (drv_status >> 31) & 0x01;
    status->stall_detected = (drv_status >> 24) & 0x01;
    status->overtemp_warning = (drv_status >> 26) & 0x01;
    status->overtemp_shutdown = (drv_status >> 25) & 0x01;
    status->short_a = (drv_status >> 27) & 0x01;
    status->short_b = (drv_status >> 28) & 0x01;
    status->open_a = (drv_status >> 29) & 0x01;
    status->open_b = (drv_status >> 30) & 0x01;
    status->sg_result = drv_status & 0x3FF;
    status->cs_actual = (drv_status >> 16) & 0x1F;
    
    return ESP_OK;
}

void tmc2209_print_status(void) {
    tmc2209_status_t status;
    
    ESP_LOGI(TAG, "=== TMC2209 Status ===");
    ESP_LOGI(TAG, "  DIAG pin (GPIO%d): %s", PIN_TMC_DIAG, 
             gpio_get_level(PIN_TMC_DIAG) ? "HIGH (stall!)" : "LOW (ok)");
    
    if (tmc2209_get_status(&status) == ESP_OK) {
        ESP_LOGI(TAG, "  [UART OK] Reading registers...");
        ESP_LOGI(TAG, "  Standstill: %s", status.standstill ? "YES" : "NO");
        ESP_LOGI(TAG, "  Stall flag: %s", status.stall_detected ? "YES" : "NO");
        ESP_LOGI(TAG, "  SG_RESULT: %d (stall when < SGTHRS*2)", status.sg_result);
        ESP_LOGI(TAG, "  CS actual: %d/31", status.cs_actual);
        ESP_LOGI(TAG, "  Errors: OT=%s Short=%s/%s Open=%s/%s", 
                 status.overtemp_warning ? "Y" : "N",
                 status.short_a ? "Y" : "N", status.short_b ? "Y" : "N",
                 status.open_a ? "Y" : "N", status.open_b ? "Y" : "N");
    } else {
        ESP_LOGW(TAG, "  [UART FAILED] Cannot read registers!");
        ESP_LOGW(TAG, "  StallGuard config may not be applied!");
        ESP_LOGW(TAG, "  Check UART wiring: TX(GPIO%d) -> TMC PDN_UART via 1k resistor", PIN_TMC_UART_TX);
    }
    
    // Also read SGTHRS to verify it was set
    uint32_t sgthrs;
    if (tmc2209_read_reg(TMC_REG_SGTHRS, &sgthrs) == ESP_OK) {
        ESP_LOGI(TAG, "  SGTHRS register: %lu (stall threshold)", (unsigned long)sgthrs);
    }
}

