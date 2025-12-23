#ifndef HTTP_H
#define HTTP_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// Lock command definitions
typedef enum {
    LOCK_CMD_NONE = 0,
    LOCK_CMD_LOCK = 1,
    LOCK_CMD_UNLOCK = 2,
    LOCK_CMD_CALIBRATE = 3,
    LOCK_CMD_STATUS = 4,
    LOCK_CMD_STOP = 5,
} lock_command_t;

// Lock status definitions
typedef enum {
    LOCK_STATUS_UNKNOWN = 0,
    LOCK_STATUS_LOCKED = 1,
    LOCK_STATUS_UNLOCKED = 2,
    LOCK_STATUS_MOVING = 3,
    LOCK_STATUS_ERROR = 4,
    LOCK_STATUS_CALIBRATING = 5,
} lock_status_t;

/**
 * @brief Initialize HTTP server
 * @return ESP_OK on success
 */
esp_err_t http_init(void);

/**
 * @brief Start HTTP server
 * @return ESP_OK on success
 */
esp_err_t http_start(void);

/**
 * @brief Stop HTTP server
 */
void http_stop(void);

/**
 * @brief Update lock status (returned by /status endpoint)
 * @param status Lock status
 * @return ESP_OK on success
 */
esp_err_t http_set_status(lock_status_t status);

/**
 * @brief Get pending command from HTTP
 * @return Command or LOCK_CMD_NONE
 */
lock_command_t http_get_pending_command(void);

/**
 * @brief Clear pending command
 */
void http_clear_command(void);

/**
 * @brief Check if HTTP server is running
 * @return true if running
 */
bool http_is_running(void);

/**
 * @brief Set callback for command received
 */
typedef void (*http_cmd_callback_t)(lock_command_t cmd);
void http_set_command_callback(http_cmd_callback_t callback);

#endif // HTTP_H

