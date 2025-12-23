#ifndef AS5600_H
#define AS5600_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// ============================================================================
// AS5600 Magnetic Rotary Position Sensor
// ============================================================================
// 
// The AS5600 is a 12-bit magnetic rotary position sensor that communicates
// via I2C. It provides absolute angle readings from 0-4095 representing
// 0-360 degrees.
//
// This driver tracks:
//   - Raw angle (0-4095)
//   - Total revolutions (counting wrap-arounds)
//   - Total angle in degrees
//
// ============================================================================

// AS5600 I2C address (fixed, not configurable)
#define AS5600_I2C_ADDR         0x36

// AS5600 Register addresses
#define AS5600_REG_ZMCO         0x00    // Zero position programming count
#define AS5600_REG_ZPOS_H       0x01    // Zero position high byte
#define AS5600_REG_ZPOS_L       0x02    // Zero position low byte
#define AS5600_REG_MPOS_H       0x03    // Maximum position high byte
#define AS5600_REG_MPOS_L       0x04    // Maximum position low byte
#define AS5600_REG_MANG_H       0x05    // Maximum angle high byte
#define AS5600_REG_MANG_L       0x06    // Maximum angle low byte
#define AS5600_REG_CONF_H       0x07    // Configuration high byte
#define AS5600_REG_CONF_L       0x08    // Configuration low byte
#define AS5600_REG_RAW_ANGLE_H  0x0C    // Raw angle high byte
#define AS5600_REG_RAW_ANGLE_L  0x0D    // Raw angle low byte
#define AS5600_REG_ANGLE_H      0x0E    // Filtered angle high byte
#define AS5600_REG_ANGLE_L      0x0F    // Filtered angle low byte
#define AS5600_REG_STATUS       0x0B    // Status register
#define AS5600_REG_AGC          0x1A    // Automatic gain control
#define AS5600_REG_MAGNITUDE_H  0x1B    // Magnitude high byte
#define AS5600_REG_MAGNITUDE_L  0x1C    // Magnitude low byte

// Status register bits
#define AS5600_STATUS_MH        0x08    // Magnet too strong
#define AS5600_STATUS_ML        0x10    // Magnet too weak
#define AS5600_STATUS_MD        0x20    // Magnet detected

// Full rotation in raw counts
#define AS5600_RAW_MAX          4096

// ============================================================================
// Revolution Tracking State
// ============================================================================

typedef struct {
    int32_t revolutions;        // Total complete revolutions (can be negative)
    uint16_t raw_angle;         // Current raw angle (0-4095)
    float total_degrees;        // Total angle including revolutions
    bool magnet_detected;       // Magnet status
} as5600_state_t;

// ============================================================================
// AS5600 API
// ============================================================================

/**
 * @brief Initialize the AS5600 sensor and I2C bus
 * @return ESP_OK on success
 */
esp_err_t as5600_init(void);

/**
 * @brief Read the raw angle value (0-4095)
 * @param raw_angle Pointer to store raw angle
 * @return ESP_OK on success
 */
esp_err_t as5600_read_raw_angle(uint16_t *raw_angle);

/**
 * @brief Read the filtered angle value (0-4095)
 * @param angle Pointer to store filtered angle
 * @return ESP_OK on success
 */
esp_err_t as5600_read_angle(uint16_t *angle);

/**
 * @brief Get current angle in degrees (0-360)
 * @param degrees Pointer to store angle in degrees
 * @return ESP_OK on success
 */
esp_err_t as5600_get_degrees(float *degrees);

/**
 * @brief Check if magnet is detected and in range
 * @param detected Pointer to store detection status
 * @return ESP_OK on success
 */
esp_err_t as5600_magnet_detected(bool *detected);

/**
 * @brief Get AGC value (automatic gain control, 0-255)
 * @param agc Pointer to store AGC value
 * @return ESP_OK on success
 */
esp_err_t as5600_get_agc(uint8_t *agc);

/**
 * @brief Get magnet magnitude value
 * @param magnitude Pointer to store magnitude
 * @return ESP_OK on success
 */
esp_err_t as5600_get_magnitude(uint16_t *magnitude);

// ============================================================================
// Revolution Tracking API
// ============================================================================

/**
 * @brief Reset revolution counter and start tracking from current position
 */
void as5600_reset_revolutions(void);

/**
 * @brief Update revolution tracking (call periodically during motion)
 * 
 * This function reads the current angle and detects wrap-arounds
 * to count complete revolutions.
 * 
 * @return ESP_OK on success
 */
esp_err_t as5600_update(void);

/**
 * @brief Get current revolution tracking state
 * @param state Pointer to store state
 * @return ESP_OK on success
 */
esp_err_t as5600_get_state(as5600_state_t *state);

/**
 * @brief Get total revolutions (including partial)
 * @return Total revolutions as float (e.g., 2.5 = 2.5 turns)
 */
float as5600_get_total_revolutions(void);

/**
 * @brief Print current status to console
 */
void as5600_print_status(void);

#endif // AS5600_H

