#ifndef TMC2209_H
#define TMC2209_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// ============================================================================
// TMC2209 Register Addresses
// ============================================================================

// General Configuration
#define TMC_REG_GCONF           0x00    // Global configuration
#define TMC_REG_GSTAT           0x01    // Global status
#define TMC_REG_IFCNT           0x02    // Interface counter
#define TMC_REG_SLAVECONF       0x03    // Slave configuration
#define TMC_REG_OTP_PROG        0x04    // OTP programming
#define TMC_REG_OTP_READ        0x05    // OTP read
#define TMC_REG_IOIN            0x06    // Input pins state
#define TMC_REG_FACTORY_CONF    0x07    // Factory configuration

// Velocity Dependent Control
#define TMC_REG_IHOLD_IRUN      0x10    // Hold/Run current
#define TMC_REG_TPOWERDOWN      0x11    // Power down delay
#define TMC_REG_TSTEP           0x12    // Microstep time
#define TMC_REG_TPWMTHRS        0x13    // StealthChop threshold
#define TMC_REG_TCOOLTHRS       0x14    // CoolStep threshold
#define TMC_REG_VACTUAL         0x22    // Actual velocity (for UART control)

// StallGuard Control
#define TMC_REG_SGTHRS          0x40    // StallGuard threshold
#define TMC_REG_SG_RESULT       0x41    // StallGuard result
#define TMC_REG_COOLCONF        0x42    // CoolStep configuration

// Sequencer Registers
#define TMC_REG_MSCNT           0x6A    // Microstep counter
#define TMC_REG_MSCURACT        0x6B    // Actual microstep current

// Chopper Control
#define TMC_REG_CHOPCONF        0x6C    // Chopper configuration
#define TMC_REG_DRVSTATUS       0x6F    // Driver status

// PWM Control
#define TMC_REG_PWMCONF         0x70    // PWM configuration

// ============================================================================
// TMC2209 Status Structure
// ============================================================================

typedef struct {
    bool standstill;            // Motor is standing still
    bool stall_detected;        // StallGuard triggered
    bool overtemp_warning;      // Temperature warning
    bool overtemp_shutdown;     // Thermal shutdown
    bool short_a;               // Short to ground coil A
    bool short_b;               // Short to ground coil B
    bool open_a;                // Open load coil A
    bool open_b;                // Open load coil B
    uint16_t sg_result;         // StallGuard result (0-510)
    uint8_t cs_actual;          // Actual current scale
} tmc2209_status_t;

// ============================================================================
// TMC2209 Configuration Structure
// ============================================================================

typedef struct {
    uint16_t run_current_ma;    // Run current in mA
    uint16_t hold_current_ma;   // Hold current in mA
    uint16_t microsteps;        // Microsteps (1, 2, 4, 8, 16, 32, 64, 128, 256)
    uint8_t stallguard_thresh;  // StallGuard threshold (0-255)
    bool stallguard_filter;     // Enable StallGuard filter
    bool stealthchop;           // Use StealthChop mode
    bool coolstep_enable;       // Enable CoolStep
    uint8_t coolstep_semin;     // CoolStep lower threshold
    uint8_t coolstep_semax;     // CoolStep upper threshold
    uint8_t coolstep_sedn;      // CoolStep decrement speed (0-3)
} tmc2209_config_t;

// ============================================================================
// TMC2209 API Functions
// ============================================================================

/**
 * @brief Initialize TMC2209 UART communication
 * @return ESP_OK on success
 */
esp_err_t tmc2209_init(void);

/**
 * @brief Apply configuration to TMC2209
 * @param config Pointer to configuration structure
 * @return ESP_OK on success
 */
esp_err_t tmc2209_configure(const tmc2209_config_t *config);

/**
 * @brief Read TMC2209 register
 * @param reg Register address
 * @param value Pointer to store read value
 * @return ESP_OK on success
 */
esp_err_t tmc2209_read_reg(uint8_t reg, uint32_t *value);

/**
 * @brief Write TMC2209 register
 * @param reg Register address
 * @param value Value to write
 * @return ESP_OK on success
 */
esp_err_t tmc2209_write_reg(uint8_t reg, uint32_t value);

/**
 * @brief Get driver status
 * @param status Pointer to status structure
 * @return ESP_OK on success
 */
esp_err_t tmc2209_get_status(tmc2209_status_t *status);

/**
 * @brief Check if stall is detected (via DIAG pin or register)
 * @return true if stall detected
 */
bool tmc2209_stall_detected(void);

/**
 * @brief Get StallGuard result value
 * @return StallGuard value (0-510), lower = higher load
 */
uint16_t tmc2209_get_sg_result(void);

/**
 * @brief Set run current
 * @param current_ma Current in milliamps
 * @return ESP_OK on success
 */
esp_err_t tmc2209_set_run_current(uint16_t current_ma);

/**
 * @brief Set hold current
 * @param current_ma Current in milliamps
 * @return ESP_OK on success
 */
esp_err_t tmc2209_set_hold_current(uint16_t current_ma);

/**
 * @brief Set microstepping resolution
 * @param microsteps Microsteps per full step (1, 2, 4, 8, 16, 32, 64, 128, 256)
 * @return ESP_OK on success
 */
esp_err_t tmc2209_set_microsteps(uint16_t microsteps);

/**
 * @brief Set StallGuard threshold
 * @param threshold Threshold value (0-255)
 * @return ESP_OK on success
 */
esp_err_t tmc2209_set_stallguard_threshold(uint8_t threshold);

/**
 * @brief Enable/disable StealthChop mode
 * @param enable true for StealthChop, false for SpreadCycle
 * @return ESP_OK on success
 */
esp_err_t tmc2209_set_stealthchop(bool enable);

/**
 * @brief Enable driver (set EN pin low)
 */
void tmc2209_enable(void);

/**
 * @brief Disable driver (set EN pin high)
 */
void tmc2209_disable(void);

/**
 * @brief Reset TMC2209 (toggle enable, clear errors, reset SGTHRS)
 */
void tmc2209_reset(void);

/**
 * @brief Set microstep resolution via MS1/MS2 pins
 * @param microsteps 8, 16, 32, or 64
 */
void tmc2209_set_ms_pins(uint16_t microsteps);

/**
 * @brief Print diagnostic information (for debugging)
 */
void tmc2209_print_status(void);

#endif // TMC2209_H

