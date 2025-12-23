#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// ============================================================================
// Lock Command Definitions (received from client)
// ============================================================================

typedef enum {
    LOCK_CMD_NONE = 0,
    LOCK_CMD_LOCK = 1,          // Lock the mechanism
    LOCK_CMD_UNLOCK = 2,        // Unlock the mechanism
    LOCK_CMD_CALIBRATE = 3,     // Run calibration
    LOCK_CMD_STATUS = 4,        // Request status
    LOCK_CMD_STOP = 5,          // Emergency stop
} lock_command_t;

// ============================================================================
// Lock Status Definitions (sent to client)
// ============================================================================

typedef enum {
    LOCK_STATUS_UNKNOWN = 0,
    LOCK_STATUS_LOCKED = 1,
    LOCK_STATUS_UNLOCKED = 2,
    LOCK_STATUS_MOVING = 3,
    LOCK_STATUS_ERROR = 4,
    LOCK_STATUS_CALIBRATING = 5,
} lock_status_t;

// ============================================================================
// Bluetooth API
// ============================================================================

/**
 * @brief Initialize Bluetooth module
 * @return ESP_OK on success
 */
esp_err_t bluetooth_init(void);

/**
 * @brief Start Bluetooth advertising
 * @return ESP_OK on success
 */
esp_err_t bluetooth_start(void);

/**
 * @brief Stop Bluetooth advertising
 * @return ESP_OK on success
 */
esp_err_t bluetooth_stop(void);

/**
 * @brief Send lock status to connected client
 * @param status Lock status value
 * @return ESP_OK on success
 */
esp_err_t bluetooth_send_status(lock_status_t status);

/**
 * @brief Send calibration progress (percentage)
 * @param progress Progress 0-100
 * @return ESP_OK on success
 */
esp_err_t bluetooth_send_progress(uint8_t progress);

/**
 * @brief Get pending command from BLE client
 * @return Command or LOCK_CMD_NONE if no pending command
 */
lock_command_t bluetooth_get_pending_command(void);

/**
 * @brief Clear pending command
 */
void bluetooth_clear_command(void);

/**
 * @brief Check if a client is connected
 * @return true if connected
 */
bool bluetooth_is_connected(void);

/**
 * @brief Set callback for command received
 * @param callback Function to call when command received
 */
typedef void (*bluetooth_cmd_callback_t)(lock_command_t cmd);
void bluetooth_set_command_callback(bluetooth_cmd_callback_t callback);

#endif // BLUETOOTH_H
