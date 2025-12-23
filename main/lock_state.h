#ifndef LOCK_STATE_H
#define LOCK_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// ============================================================================
// Lock State Machine
// ============================================================================
//
// This module manages the smart lock state using the AS5600 encoder for
// absolute position tracking. It handles:
//   - Continuous position monitoring (detects key usage)
//   - Calibration to find the lock position
//   - Angle-based lock/unlock operations
//   - Magnet orientation compensation
//
// ============================================================================

// Lock states
typedef enum {
    LOCK_STATE_UNCALIBRATED,    // Not yet calibrated - position unknown
    LOCK_STATE_LOCKED,          // At or near the calibrated lock position
    LOCK_STATE_UNLOCKED,        // One full rotation from lock position
    LOCK_STATE_MID,             // Between locked and unlocked
    LOCK_STATE_MOVING,          // Motor is actively moving
    LOCK_STATE_CALIBRATING,     // Calibration in progress
    LOCK_STATE_ERROR            // Error state
} lock_state_t;

// Magnet direction - which way angle increases when motor turns CW
typedef enum {
    MAGNET_DIR_UNKNOWN = 0,
    MAGNET_DIR_CW_POSITIVE = 1,   // CW motor rotation = increasing angle
    MAGNET_DIR_CW_NEGATIVE = -1   // CW motor rotation = decreasing angle
} magnet_direction_t;

// Complete state data structure
typedef struct {
    lock_state_t state;             // Current state
    bool calibrated;                // Has calibration been performed
    float lock_angle_deg;           // Calibrated lock position (0-360)
    float unlock_angle_deg;         // Calculated unlock position
    float current_angle_deg;        // Current encoder angle (0-360)
    float angle_from_lock;          // Signed degrees from lock position
    magnet_direction_t magnet_dir;  // Magnet orientation
} lock_state_data_t;

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize lock state tracking
 * 
 * Sets up background task for continuous encoder monitoring.
 * Call after as5600_init() and stepper_init().
 * 
 * @return ESP_OK on success
 */
esp_err_t lock_state_init(void);

// ============================================================================
// State Access
// ============================================================================

/**
 * @brief Get current lock state data
 * @param data Pointer to store state data
 * @return ESP_OK on success
 */
esp_err_t lock_state_get(lock_state_data_t *data);

/**
 * @brief Get current state as string
 * @return String representation of current state
 */
const char* lock_state_get_string(void);

/**
 * @brief Get magnet direction as string
 * @return "cw", "ccw", or "unknown"
 */
const char* lock_state_get_magnet_dir_string(void);

// ============================================================================
// Calibration
// ============================================================================

/**
 * @brief Run calibration routine
 * 
 * 1. Rotates motor CW until stall (finds lock position)
 * 2. Backs off by LOCK_BACKOFF_DEG degrees
 * 3. Saves current encoder angle as lock position
 * 4. Detects magnet orientation
 * 
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if no stall detected
 */
esp_err_t lock_state_calibrate(void);

/**
 * @brief Check if system is calibrated
 * @return true if calibrated
 */
bool lock_state_is_calibrated(void);

// ============================================================================
// Lock/Unlock Operations
// ============================================================================

/**
 * @brief Move to lock position
 * 
 * Calculates shortest path to lock angle and moves motor.
 * Uses encoder feedback for positioning.
 * 
 * @return ESP_OK on success
 */
esp_err_t lock_state_move_to_lock(void);

/**
 * @brief Move to unlock position
 * 
 * Rotates one full turn (360 degrees) from lock position.
 * Direction determined by magnet orientation.
 * 
 * @return ESP_OK on success
 */
esp_err_t lock_state_move_to_unlock(void);

// ============================================================================
// Status/Debug
// ============================================================================

/**
 * @brief Print current state to console
 */
void lock_state_print_status(void);

/**
 * @brief Get JSON status string (caller must free)
 * @return Allocated JSON string or NULL on error
 */
char* lock_state_get_json_status(void);

#endif // LOCK_STATE_H

