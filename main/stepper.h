#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// ============================================================================
// Stepper Motor State
// ============================================================================

typedef enum {
    STEPPER_STATE_IDLE,         // Motor idle/disabled
    STEPPER_STATE_RUNNING,      // Motor moving
    STEPPER_STATE_HOMING,       // Homing in progress
    STEPPER_STATE_STALLED,      // Stall detected
    STEPPER_STATE_ERROR,        // Error state
} stepper_state_t;

typedef enum {
    STEPPER_DIR_CW = 0,         // Clockwise (lock)
    STEPPER_DIR_CCW = 1,        // Counter-clockwise (unlock)
} stepper_dir_t;

// ============================================================================
// Lock Position State
// ============================================================================

typedef enum {
    LOCK_POSITION_UNKNOWN,      // Position not calibrated
    LOCK_POSITION_LOCKED,       // At locked position
    LOCK_POSITION_UNLOCKED,     // At unlocked position
    LOCK_POSITION_MOVING,       // Moving between positions
} lock_position_t;

// ============================================================================
// Stepper Configuration
// ============================================================================

typedef struct {
    uint32_t max_speed_hz;      // Maximum step frequency
    uint32_t accel_hz_per_s;    // Acceleration rate
    uint32_t homing_speed_hz;   // Speed during homing
    int32_t travel_steps;       // Steps between locked/unlocked
    uint32_t homing_timeout_ms; // Maximum homing time
    int32_t backoff_steps;      // Steps to back off after stall
} stepper_config_t;

// ============================================================================
// Stepper Status
// ============================================================================

typedef struct {
    stepper_state_t state;
    lock_position_t position;
    int32_t current_step;       // Current step position
    int32_t target_step;        // Target step position
    bool is_homed;              // Has been calibrated
    bool stall_detected;        // Stall flag
    uint32_t steps_moved;       // Total steps moved (for diagnostics)
} stepper_status_t;

// ============================================================================
// Stepper Motor API
// ============================================================================

/**
 * @brief Initialize stepper motor subsystem
 * @return ESP_OK on success
 */
esp_err_t stepper_init(void);

/**
 * @brief Set stepper configuration
 * @param config Pointer to configuration structure
 * @return ESP_OK on success
 */
esp_err_t stepper_set_config(const stepper_config_t *config);

/**
 * @brief Get current stepper status
 * @param status Pointer to status structure
 * @return ESP_OK on success
 */
esp_err_t stepper_get_status(stepper_status_t *status);

/**
 * @brief Enable stepper motor (power on)
 */
void stepper_enable(void);

/**
 * @brief Disable stepper motor (power off)
 */
void stepper_disable(void);

/**
 * @brief Move stepper by relative steps
 * @param steps Number of steps (positive = CW, negative = CCW)
 * @param wait Wait for move to complete
 * @return ESP_OK on success, ESP_ERR_TIMEOUT on stall
 */
esp_err_t stepper_move_steps(int32_t steps, bool wait);

/**
 * @brief Move stepper to absolute position
 * @param position Target position in steps
 * @param wait Wait for move to complete
 * @return ESP_OK on success
 */
esp_err_t stepper_move_to(int32_t position, bool wait);

/**
 * @brief Stop motor immediately
 */
void stepper_stop(void);

/**
 * @brief Home the stepper using StallGuard
 * 
 * Moves in specified direction until stall is detected, then sets
 * that position as the home position (0).
 * 
 * @param direction Direction to home
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if no stall detected
 */
esp_err_t stepper_home(stepper_dir_t direction);

/**
 * @brief Find both end positions using StallGuard
 * 
 * 1. Home in one direction (find lock position)
 * 2. Move other direction until stall (find unlock position)
 * 3. Calculate total travel
 * 
 * @return ESP_OK on success
 */
esp_err_t stepper_calibrate(void);

/**
 * @brief Check if motor is currently moving
 * @return true if moving
 */
bool stepper_is_moving(void);

/**
 * @brief Wait for current move to complete
 * @param timeout_ms Maximum time to wait
 * @return ESP_OK on completion, ESP_ERR_TIMEOUT on timeout
 */
esp_err_t stepper_wait_idle(uint32_t timeout_ms);

/**
 * @brief Set current position as zero
 */
void stepper_set_zero(void);

/**
 * @brief Clear stall flag
 */
void stepper_clear_stall(void);

// ============================================================================
// Lock-Specific Functions
// ============================================================================

/**
 * @brief Move to locked position
 * @return ESP_OK on success
 */
esp_err_t lock_engage(void);

/**
 * @brief Move to unlocked position
 * @return ESP_OK on success
 */
esp_err_t lock_disengage(void);

/**
 * @brief Get lock position state
 * @return Lock position enum
 */
lock_position_t lock_get_position(void);

/**
 * @brief Check if lock is calibrated
 * @return true if calibrated
 */
bool lock_is_calibrated(void);

#endif // STEPPER_H

