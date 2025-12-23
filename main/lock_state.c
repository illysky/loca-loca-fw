#include "lock_state.h"
#include "config.h"
#include "as5600.h"
#include "stepper.h"
#include "tmc2209.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "cJSON.h"
#include <math.h>
#include <string.h>

static const char *TAG = "LOCK_STATE";

// ============================================================================
// Internal State
// ============================================================================

static struct {
    lock_state_t state;
    bool calibrated;
    float lock_angle_deg;           // Calibrated lock position
    float unlock_angle_deg;         // Calculated unlock position
    float current_angle_deg;        // Current encoder angle
    float last_angle_deg;           // Previous angle (for key detection)
    magnet_direction_t magnet_dir;
    SemaphoreHandle_t mutex;
    TaskHandle_t tracking_task;
    bool motor_active;              // True when motor is being commanded
} state = {
    .state = LOCK_STATE_UNCALIBRATED,
    .calibrated = false,
    .lock_angle_deg = 0,
    .unlock_angle_deg = 0,
    .current_angle_deg = 0,
    .last_angle_deg = 0,
    .magnet_dir = MAGNET_DIR_UNKNOWN,
    .mutex = NULL,
    .tracking_task = NULL,
    .motor_active = false
};

// ============================================================================
// Helper Functions
// ============================================================================

// Normalize angle to 0-360 range
static float normalize_angle(float angle) {
    while (angle < 0) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

// Calculate signed angle difference (shortest path)
static float angle_diff(float from, float to) {
    float diff = to - from;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

// Read current encoder angle in degrees (0-360)
static float read_encoder_degrees(void) {
    float deg;
    if (as5600_get_degrees(&deg) == ESP_OK) {
        return deg;
    }
    return 0;
}

// Determine state based on current angle
static lock_state_t determine_state_from_angle(float current, float lock_angle, float tolerance) {
    if (!state.calibrated) {
        return LOCK_STATE_UNCALIBRATED;
    }
    
    float diff = fabsf(angle_diff(current, lock_angle));
    
    if (diff <= tolerance) {
        return LOCK_STATE_LOCKED;
    }
    
    // Check if at unlock position (360 degrees from lock)
    float unlock_diff = fabsf(angle_diff(current, state.unlock_angle_deg));
    if (unlock_diff <= tolerance) {
        return LOCK_STATE_UNLOCKED;
    }
    
    return LOCK_STATE_MID;
}

// ============================================================================
// Background Tracking Task
// ============================================================================

static void tracking_task(void *pvParameters) {
    ESP_LOGI(TAG, "Position tracking task started");
    
    TickType_t last_wake = xTaskGetTickCount();
    
    while (1) {
        // Read current angle
        float current = read_encoder_degrees();
        
        xSemaphoreTake(state.mutex, portMAX_DELAY);
        
        state.current_angle_deg = current;
        
        // Detect key usage (position changed while motor not active)
        if (!state.motor_active && state.calibrated) {
            float delta = fabsf(angle_diff(current, state.last_angle_deg));
            if (delta > 5.0f) {  // More than 5 degrees = key was used
                ESP_LOGI(TAG, "Key usage detected! Angle changed %.1f -> %.1f (delta: %.1f)",
                         state.last_angle_deg, current, delta);
            }
        }
        
        state.last_angle_deg = current;
        
        // Update state if not currently moving
        if (state.state != LOCK_STATE_MOVING && 
            state.state != LOCK_STATE_CALIBRATING) {
            state.state = determine_state_from_angle(
                current, state.lock_angle_deg, LOCK_ANGLE_TOLERANCE_DEG);
        }
        
        xSemaphoreGive(state.mutex);
        
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ENCODER_POLL_INTERVAL_MS));
    }
}

// ============================================================================
// Initialization
// ============================================================================

esp_err_t lock_state_init(void) {
    ESP_LOGI(TAG, "Initializing lock state tracking");
    
    state.mutex = xSemaphoreCreateMutex();
    if (state.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Read initial angle
    state.current_angle_deg = read_encoder_degrees();
    state.last_angle_deg = state.current_angle_deg;
    
    // Start background tracking task
    BaseType_t ret = xTaskCreate(
        tracking_task,
        "lock_track",
        4096,
        NULL,
        3,  // Lower priority than main task
        &state.tracking_task
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create tracking task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Lock state initialized - current angle: %.1f°", state.current_angle_deg);
    return ESP_OK;
}

// ============================================================================
// State Access
// ============================================================================

esp_err_t lock_state_get(lock_state_data_t *data) {
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(state.mutex, portMAX_DELAY);
    
    data->state = state.state;
    data->calibrated = state.calibrated;
    data->lock_angle_deg = state.lock_angle_deg;
    data->unlock_angle_deg = state.unlock_angle_deg;
    data->current_angle_deg = state.current_angle_deg;
    data->angle_from_lock = state.calibrated ? 
        angle_diff(state.lock_angle_deg, state.current_angle_deg) : 0;
    data->magnet_dir = state.magnet_dir;
    
    xSemaphoreGive(state.mutex);
    
    return ESP_OK;
}

const char* lock_state_get_string(void) {
    switch (state.state) {
        case LOCK_STATE_UNCALIBRATED: return "uncalibrated";
        case LOCK_STATE_LOCKED:       return "locked";
        case LOCK_STATE_UNLOCKED:     return "unlocked";
        case LOCK_STATE_MID:          return "mid";
        case LOCK_STATE_MOVING:       return "moving";
        case LOCK_STATE_CALIBRATING:  return "calibrating";
        case LOCK_STATE_ERROR:        return "error";
        default:                      return "unknown";
    }
}

const char* lock_state_get_magnet_dir_string(void) {
    switch (state.magnet_dir) {
        case MAGNET_DIR_CW_POSITIVE: return "cw";
        case MAGNET_DIR_CW_NEGATIVE: return "ccw";
        default:                     return "unknown";
    }
}

bool lock_state_is_calibrated(void) {
    return state.calibrated;
}

// ============================================================================
// Calibration
// ============================================================================

esp_err_t lock_state_calibrate(void) {
    ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║      STARTING CALIBRATION             ║");
    ESP_LOGI(TAG, "║   (Using encoder stall detection)     ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
    
    xSemaphoreTake(state.mutex, portMAX_DELAY);
    state.state = LOCK_STATE_CALIBRATING;
    state.motor_active = true;
    xSemaphoreGive(state.mutex);
    
    // Read starting angle
    float start_angle = read_encoder_degrees();
    ESP_LOGI(TAG, "Starting angle: %.1f°", start_angle);
    
    // Enable motor
    stepper_enable();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Rotate CCW until encoder stops moving (this finds the lock position)
    ESP_LOGI(TAG, "Rotating CCW to find lock position...");
    gpio_set_level(PIN_TMC_DIR, STEPPER_DIR_CCW);
    
    int32_t max_steps = LOCK_MAX_STEPS * 2;
    bool stall_detected = false;
    int32_t steps_taken = 0;
    
    uint32_t step_delay_us = 1000000 / CALIBRATE_SPEED_HZ;
    
    // Encoder-based stall detection (using config values)
    float last_check_angle = start_angle;
    int stall_count = 0;
    
    while (steps_taken < max_steps && !stall_detected) {
        // Generate step
        gpio_set_level(PIN_TMC_STEP, 1);
        esp_rom_delay_us(10);
        gpio_set_level(PIN_TMC_STEP, 0);
        esp_rom_delay_us(step_delay_us);
        
        steps_taken++;
        
        // Check encoder every 100 steps
        if (steps_taken % 100 == 0 && steps_taken > 200) {
            float current_angle = read_encoder_degrees();
            float movement = fabsf(angle_diff(last_check_angle, current_angle));
            
            if (movement < STALL_MIN_MOVEMENT_DEG) {
                stall_count++;
                ESP_LOGI(TAG, "No movement detected (%d/%d), angle=%.1f°", 
                         stall_count, STALL_COUNT_THRESHOLD, current_angle);
                if (stall_count >= STALL_COUNT_THRESHOLD) {
                    ESP_LOGI(TAG, "Stall detected at step %ld - encoder stopped moving", (long)steps_taken);
                    stall_detected = true;
                }
            } else {
                stall_count = 0;  // Reset if we see movement
            }
            last_check_angle = current_angle;
        }
    }
    
    if (!stall_detected) {
        ESP_LOGE(TAG, "Calibration failed - encoder never stopped");
        stepper_disable();
        
        xSemaphoreTake(state.mutex, portMAX_DELAY);
        state.state = LOCK_STATE_ERROR;
        state.motor_active = false;
        xSemaphoreGive(state.mutex);
        
        return ESP_ERR_TIMEOUT;
    }
    
    // Read angle at stall - this is our lock position
    float stall_angle = read_encoder_degrees();
    ESP_LOGI(TAG, "Lock position found at: %.1f°", stall_angle);
    
    // Back off slightly from the hard stop
    ESP_LOGI(TAG, "Backing off %.1f degrees...", LOCK_BACKOFF_DEG);
    gpio_set_level(PIN_TMC_DIR, STEPPER_DIR_CW);  // Reverse direction
    
    as5600_reset_revolutions();
    int backoff_steps = 0;
    int max_backoff = 500;
    // Back off by a fixed number of degrees using encoder
    float backoff_target = LOCK_BACKOFF_DEG / 360.0f;  // Target revolutions
    while (backoff_steps < max_backoff) {
        gpio_set_level(PIN_TMC_STEP, 1);
        esp_rom_delay_us(10);
        gpio_set_level(PIN_TMC_STEP, 0);
        esp_rom_delay_us(step_delay_us);
        backoff_steps++;
        
        if (backoff_steps % 20 == 0) {
            as5600_update();
            float revs = fabsf(as5600_get_total_revolutions());
            if (revs >= backoff_target) {
                break;
            }
        }
    }
    
    // Final position is our lock angle
    float lock_angle = read_encoder_degrees();
    // Unlock angle is UNLOCK_ROTATION_DEG CW from lock
    float unlock_angle = normalize_angle(lock_angle + UNLOCK_ROTATION_DEG);
    
    ESP_LOGI(TAG, "Calibration complete!");
    ESP_LOGI(TAG, "  Lock angle:   %.1f°", lock_angle);
    ESP_LOGI(TAG, "  Unlock angle: %.1f°", unlock_angle);
    
    // Disable motor
    stepper_disable();
    
    // Update state
    xSemaphoreTake(state.mutex, portMAX_DELAY);
    state.calibrated = true;
    state.lock_angle_deg = lock_angle;
    state.unlock_angle_deg = unlock_angle;
    state.magnet_dir = MAGNET_DIR_CW_POSITIVE;  // Not used anymore but keep for compatibility
    state.state = LOCK_STATE_LOCKED;
    state.motor_active = false;
    xSemaphoreGive(state.mutex);
    
    ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║      CALIBRATION COMPLETE             ║");
    ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
    
    return ESP_OK;
}

// ============================================================================
// Lock/Unlock Operations
// ============================================================================

esp_err_t lock_state_move_to_lock(void) {
    if (!state.calibrated) {
        ESP_LOGE(TAG, "Cannot lock - not calibrated");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Moving to LOCK position...");
    
    xSemaphoreTake(state.mutex, portMAX_DELAY);
    state.state = LOCK_STATE_MOVING;
    state.motor_active = true;
    float target = state.lock_angle_deg;
    float start_angle = state.current_angle_deg;
    xSemaphoreGive(state.mutex);
    
    ESP_LOGI(TAG, "Current: %.1f°, Target: %.1f°", start_angle, target);
    
    if (fabsf(angle_diff(start_angle, target)) < LOCK_ANGLE_TOLERANCE_DEG) {
        ESP_LOGI(TAG, "Already at lock position");
        xSemaphoreTake(state.mutex, portMAX_DELAY);
        state.state = LOCK_STATE_LOCKED;
        state.motor_active = false;
        xSemaphoreGive(state.mutex);
        return ESP_OK;
    }
    
    // Enable motor
    stepper_enable();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Lock always goes CCW (same direction calibration used to find lock position)
    stepper_dir_t motor_dir = STEPPER_DIR_CCW;
    gpio_set_level(PIN_TMC_DIR, motor_dir);
    
    // Move until we reach target angle, using encoder-based stall detection
    uint32_t step_delay_us = 1000000 / LOCK_SPEED_HZ;
    int steps = 0;
    int max_steps = LOCK_MAX_STEPS;
    bool stall_detected = false;
    
    // Encoder-based stall detection (using config values)
    float last_check_angle = start_angle;
    int stall_count = 0;
    
    while (steps < max_steps) {
        // Generate step
        gpio_set_level(PIN_TMC_STEP, 1);
        esp_rom_delay_us(10);
        gpio_set_level(PIN_TMC_STEP, 0);
        esp_rom_delay_us(step_delay_us);
        steps++;
        
        // Check position periodically
        if (steps % 50 == 0) {
            float curr = read_encoder_degrees();
            float diff = fabsf(angle_diff(curr, target));
            
            if (diff < LOCK_ANGLE_TOLERANCE_DEG) {
                ESP_LOGI(TAG, "Reached lock position at %.1f° (target: %.1f°)", curr, target);
                break;
            }
            
            // Encoder-based stall detection (lever not down!)
            float movement = fabsf(angle_diff(last_check_angle, curr));
            if (movement < STALL_MIN_MOVEMENT_DEG && steps > 200) {
                stall_count++;
                if (stall_count >= STALL_COUNT_THRESHOLD) {
                    ESP_LOGW(TAG, "⚠️ STALL detected during lock - lever not down! (encoder stopped)");
                    stall_detected = true;
                    break;
                }
            } else {
                stall_count = 0;
            }
            last_check_angle = curr;
        }
    }
    
    // Check final position
    float final_angle = read_encoder_degrees();
    float final_diff = fabsf(angle_diff(final_angle, target));
    
    // If stall detected or didn't reach target, return to start position
    if (stall_detected || final_diff >= LOCK_ANGLE_TOLERANCE_DEG) {
        ESP_LOGW(TAG, "Lock failed - returning to start position (%.1f°)", start_angle);
        
        // Reverse direction to go back
        gpio_set_level(PIN_TMC_DIR, !motor_dir);
        vTaskDelay(pdMS_TO_TICKS(50));
        
        // Move back toward start position
        int return_steps = 0;
        int max_return = LOCK_MAX_STEPS;
        while (return_steps < max_return) {
            gpio_set_level(PIN_TMC_STEP, 1);
            esp_rom_delay_us(10);
            gpio_set_level(PIN_TMC_STEP, 0);
            esp_rom_delay_us(step_delay_us);
            return_steps++;
            
            if (return_steps % 50 == 0) {
                float curr = read_encoder_degrees();
                float diff = fabsf(angle_diff(curr, start_angle));
                if (diff < LOCK_ANGLE_TOLERANCE_DEG) {
                    ESP_LOGI(TAG, "Returned to start position at %.1f°", curr);
                    break;
                }
            }
        }
        
        stepper_disable();
        
        xSemaphoreTake(state.mutex, portMAX_DELAY);
        state.motor_active = false;
        state.current_angle_deg = read_encoder_degrees();
        state.state = LOCK_STATE_UNLOCKED;  // Remain unlocked
        xSemaphoreGive(state.mutex);
        
        ESP_LOGE(TAG, "❌ Lock FAILED - lever not down! Returned to unlocked position.");
        return ESP_FAIL;  // Return failure so main.c can flash LED
    }
    
    stepper_disable();
    
    xSemaphoreTake(state.mutex, portMAX_DELAY);
    state.motor_active = false;
    state.current_angle_deg = final_angle;
    state.state = LOCK_STATE_LOCKED;
    xSemaphoreGive(state.mutex);
    
    ESP_LOGI(TAG, "✅ LOCKED at %.1f°", final_angle);
    return ESP_OK;
}

esp_err_t lock_state_move_to_unlock(void) {
    if (!state.calibrated) {
        ESP_LOGE(TAG, "Cannot unlock - not calibrated");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Moving to UNLOCK position (360° rotation)...");
    
    xSemaphoreTake(state.mutex, portMAX_DELAY);
    state.state = LOCK_STATE_MOVING;
    state.motor_active = true;
    float current = state.current_angle_deg;
    xSemaphoreGive(state.mutex);
    
    ESP_LOGI(TAG, "Unlocking from %.1f°, rotating %.1f°", current, UNLOCK_ROTATION_DEG);
    
    // Enable motor
    stepper_enable();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Unlock always goes CW (opposite of lock/calibration direction)
    stepper_dir_t motor_dir = STEPPER_DIR_CW;
    gpio_set_level(PIN_TMC_DIR, motor_dir);
    
    // Reset encoder revolution counter to track how far we've gone
    as5600_reset_revolutions();
    
    // Move until we've rotated UNLOCK_ROTATION_DEG
    uint32_t step_delay_us = 1000000 / LOCK_SPEED_HZ;
    int steps = 0;
    int max_steps = LOCK_MAX_STEPS * 2;
    float target_revs = UNLOCK_ROTATION_DEG / 360.0f;
    
    // Encoder-based stall detection (using config values)
    float last_revs = 0;
    int stall_count = 0;
    const float MIN_MOVEMENT_REVS = STALL_MIN_MOVEMENT_DEG / 360.0f;
    
    while (steps < max_steps) {
        // Generate step
        gpio_set_level(PIN_TMC_STEP, 1);
        esp_rom_delay_us(10);
        gpio_set_level(PIN_TMC_STEP, 0);
        esp_rom_delay_us(step_delay_us);
        steps++;
        
        // Check position periodically
        if (steps % 50 == 0) {
            as5600_update();
            float revs = fabsf(as5600_get_total_revolutions());
            
            // Check if we've reached the unlock rotation
            if (revs >= (target_revs - 0.03f)) {
                ESP_LOGI(TAG, "Reached unlock position - %.2f revolutions (target: %.2f)", revs, target_revs);
                break;
            }
            
            // Encoder-based stall detection (safety)
            float movement = fabsf(revs - last_revs);
            if (movement < MIN_MOVEMENT_REVS && steps > 500) {
                stall_count++;
                if (stall_count >= STALL_COUNT_THRESHOLD) {
                    ESP_LOGW(TAG, "Stall detected during unlock (encoder stopped) at revs=%.2f", revs);
                    break;
                }
            } else {
                stall_count = 0;
            }
            last_revs = revs;
        }
    }
    
    float final_revs = fabsf(as5600_get_total_revolutions());
    stepper_disable();
    
    xSemaphoreTake(state.mutex, portMAX_DELAY);
    state.motor_active = false;
    float final_angle = read_encoder_degrees();
    state.current_angle_deg = final_angle;
    
    // Determine final state (target_revs already defined above)
    if (final_revs >= (target_revs - 0.05f)) {  // Within ~18° of target
        state.state = LOCK_STATE_UNLOCKED;
        ESP_LOGI(TAG, "✅ UNLOCKED at %.1f° (%.2f revolutions)", final_angle, final_revs);
    } else {
        state.state = LOCK_STATE_MID;
        ESP_LOGW(TAG, "Unlock incomplete - only %.2f revolutions (target: %.2f)", final_revs, target_revs);
    }
    xSemaphoreGive(state.mutex);
    
    return ESP_OK;
}

// ============================================================================
// Status/Debug
// ============================================================================

void lock_state_print_status(void) {
    lock_state_data_t data;
    lock_state_get(&data);
    
    ESP_LOGI(TAG, "┌─────────────────────────────────────────┐");
    ESP_LOGI(TAG, "│          LOCK STATE STATUS              │");
    ESP_LOGI(TAG, "├─────────────────────────────────────────┤");
    ESP_LOGI(TAG, "│ State:         %-20s    │", lock_state_get_string());
    ESP_LOGI(TAG, "│ Calibrated:    %-20s    │", data.calibrated ? "YES" : "NO");
    if (data.calibrated) {
        ESP_LOGI(TAG, "│ Lock angle:    %6.1f°                  │", data.lock_angle_deg);
        ESP_LOGI(TAG, "│ Unlock angle:  %6.1f°                  │", data.unlock_angle_deg);
        ESP_LOGI(TAG, "│ Current angle: %6.1f°                  │", data.current_angle_deg);
        ESP_LOGI(TAG, "│ From lock:     %+6.1f°                  │", data.angle_from_lock);
        ESP_LOGI(TAG, "│ Magnet dir:    %-20s    │", lock_state_get_magnet_dir_string());
    }
    ESP_LOGI(TAG, "└─────────────────────────────────────────┘");
}

char* lock_state_get_json_status(void) {
    lock_state_data_t data;
    lock_state_get(&data);
    
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return NULL;
    }
    
    cJSON_AddStringToObject(root, "state", lock_state_get_string());
    cJSON_AddBoolToObject(root, "calibrated", data.calibrated);
    cJSON_AddNumberToObject(root, "current_angle", data.current_angle_deg);
    
    if (data.calibrated) {
        cJSON_AddNumberToObject(root, "lock_angle", data.lock_angle_deg);
        cJSON_AddNumberToObject(root, "unlock_angle", data.unlock_angle_deg);
        cJSON_AddNumberToObject(root, "angle_from_lock", data.angle_from_lock);
        cJSON_AddStringToObject(root, "magnet_direction", lock_state_get_magnet_dir_string());
    }
    
    char *json_str = cJSON_Print(root);
    cJSON_Delete(root);
    
    return json_str;
}

