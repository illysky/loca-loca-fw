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
    ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
    
    xSemaphoreTake(state.mutex, portMAX_DELAY);
    state.state = LOCK_STATE_CALIBRATING;
    state.motor_active = true;
    xSemaphoreGive(state.mutex);
    
    // Read starting angle
    float start_angle = read_encoder_degrees();
    ESP_LOGI(TAG, "Starting angle: %.1f°", start_angle);
    
    // Set sensitive StallGuard threshold for calibration
    ESP_LOGI(TAG, "Setting calibration StallGuard threshold: %d", TMC_STALLGUARD_THRESH_CALIBRATE);
    tmc2209_write_reg(0x40, TMC_STALLGUARD_THRESH_CALIBRATE);
    
    // Enable motor
    stepper_enable();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Set up TCOOLTHRS for StallGuard
    tmc2209_write_reg(0x14, 0xFFFFF);
    
    // Rotate CCW until stall (this finds the lock position)
    ESP_LOGI(TAG, "Rotating CCW to find lock position...");
    gpio_set_level(PIN_TMC_DIR, STEPPER_DIR_CCW);
    
    // Start moving - we'll watch for stall
    int32_t max_steps = LOCK_MAX_STEPS * 2;  // Allow more travel for calibration
    bool stall_detected = false;
    int32_t steps_taken = 0;
    
    // Use a slower speed for calibration
    uint32_t step_delay_us = 1000000 / CALIBRATE_SPEED_HZ;  // Slower for reliable stall detection
    
    while (steps_taken < max_steps && !stall_detected) {
        // Generate step
        gpio_set_level(PIN_TMC_STEP, 1);
        esp_rom_delay_us(10);
        gpio_set_level(PIN_TMC_STEP, 0);
        esp_rom_delay_us(step_delay_us);
        
        steps_taken++;
        
        // Check StallGuard every 100 steps
        if (steps_taken % 100 == 0) {
            uint16_t sg = tmc2209_get_sg_result();
            uint16_t threshold = TMC_STALLGUARD_THRESH_CALIBRATE * 2;
            
            if (sg < threshold && sg != 0xFFFF && steps_taken > 500) {
                ESP_LOGI(TAG, "Stall detected at step %ld, SG=%d", (long)steps_taken, sg);
                stall_detected = true;
            }
        }
    }
    
    if (!stall_detected) {
        ESP_LOGE(TAG, "Calibration failed - no stall detected");
        stepper_disable();
        
        xSemaphoreTake(state.mutex, portMAX_DELAY);
        state.state = LOCK_STATE_ERROR;
        state.motor_active = false;
        xSemaphoreGive(state.mutex);
        
        return ESP_ERR_TIMEOUT;
    }
    
    // Read angle at stall
    float stall_angle = read_encoder_degrees();
    ESP_LOGI(TAG, "Stall angle: %.1f°", stall_angle);
    
    // Determine magnet direction (we moved CCW to get here)
    float angle_change = angle_diff(start_angle, stall_angle);
    magnet_direction_t detected_dir;
    if (angle_change < 0) {
        detected_dir = MAGNET_DIR_CW_POSITIVE;
        ESP_LOGI(TAG, "Magnet direction: CW = increasing angle (CCW decreased)");
    } else {
        detected_dir = MAGNET_DIR_CW_NEGATIVE;
        ESP_LOGI(TAG, "Magnet direction: CW = decreasing angle (CCW increased)");
    }
    
    // Back off from the stall
    ESP_LOGI(TAG, "Backing off %.1f degrees...", LOCK_BACKOFF_DEG);
    gpio_set_level(PIN_TMC_DIR, STEPPER_DIR_CW);  // Reverse (was CCW, now CW)
    
    float target_angle;
    if (detected_dir == MAGNET_DIR_CW_POSITIVE) {
        // We went CCW (decreasing), backoff by going CW (increasing)
        target_angle = normalize_angle(stall_angle + LOCK_BACKOFF_DEG);
    } else {
        // We went CCW (increasing), backoff by going CW (decreasing)
        target_angle = normalize_angle(stall_angle - LOCK_BACKOFF_DEG);
    }
    
    // Move until we reach backoff position
    int backoff_steps = 0;
    int max_backoff = 1000;
    while (backoff_steps < max_backoff) {
        gpio_set_level(PIN_TMC_STEP, 1);
        esp_rom_delay_us(10);
        gpio_set_level(PIN_TMC_STEP, 0);
        esp_rom_delay_us(step_delay_us);
        backoff_steps++;
        
        if (backoff_steps % 50 == 0) {
            float curr = read_encoder_degrees();
            float diff = fabsf(angle_diff(curr, target_angle));
            if (diff < 2.0f) {
                break;
            }
        }
    }
    
    // Final position is our lock angle
    float lock_angle = read_encoder_degrees();
    // Unlock is 360° CW from lock (opposite direction we came from)
    float unlock_angle = normalize_angle(lock_angle + 
        (detected_dir == MAGNET_DIR_CW_POSITIVE ? UNLOCK_ROTATION_DEG : -UNLOCK_ROTATION_DEG));
    
    ESP_LOGI(TAG, "Calibration complete!");
    ESP_LOGI(TAG, "  Lock angle:   %.1f°", lock_angle);
    ESP_LOGI(TAG, "  Unlock angle: %.1f°", unlock_angle);
    
    // Restore normal StallGuard threshold
    tmc2209_write_reg(0x40, TMC_STALLGUARD_THRESH_OPERATE);
    
    // Disable motor
    stepper_disable();
    
    // Update state
    xSemaphoreTake(state.mutex, portMAX_DELAY);
    state.calibrated = true;
    state.lock_angle_deg = lock_angle;
    state.unlock_angle_deg = unlock_angle;
    state.magnet_dir = detected_dir;
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
    
    // Set StallGuard for normal operation
    tmc2209_write_reg(0x40, TMC_STALLGUARD_THRESH_OPERATE);
    tmc2209_write_reg(0x14, 0xFFFFF);
    
    // Enable motor
    stepper_enable();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Lock always goes CCW (same direction calibration used to find lock position)
    stepper_dir_t motor_dir = STEPPER_DIR_CCW;
    gpio_set_level(PIN_TMC_DIR, motor_dir);
    
    // Move until we reach target angle
    uint32_t step_delay_us = 1000000 / LOCK_SPEED_HZ;
    int steps = 0;
    int max_steps = LOCK_MAX_STEPS;
    bool stall_detected = false;
    
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
            
            // Check for stall (lever not down!)
            uint16_t sg = tmc2209_get_sg_result();
            if (sg < TMC_STALLGUARD_THRESH_OPERATE * 2 && sg != 0xFFFF && steps > 200) {
                ESP_LOGW(TAG, "⚠️ STALL detected during lock at step %d - lever not down!", steps);
                stall_detected = true;
                break;
            }
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
    
    // Set StallGuard for normal operation
    tmc2209_write_reg(0x40, TMC_STALLGUARD_THRESH_OPERATE);
    tmc2209_write_reg(0x14, 0xFFFFF);
    
    // Enable motor
    stepper_enable();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Unlock always goes CW (opposite of lock/calibration direction)
    stepper_dir_t motor_dir = STEPPER_DIR_CW;
    gpio_set_level(PIN_TMC_DIR, motor_dir);
    
    // Reset encoder revolution counter to track how far we've gone
    as5600_reset_revolutions();
    
    // Move until we've gone approximately 360 degrees
    uint32_t step_delay_us = 1000000 / LOCK_SPEED_HZ;
    int steps = 0;
    int max_steps = LOCK_MAX_STEPS * 2;  // Allow extra for a full rotation
    
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
            float target_revs = UNLOCK_ROTATION_DEG / 360.0f;  // 350° = 0.972 revs
            
            // Check if we've reached the unlock rotation
            if (revs >= (target_revs - 0.03f)) {  // Within ~10° of target
                ESP_LOGI(TAG, "Reached unlock position - %.2f revolutions (target: %.2f)", revs, target_revs);
                break;
            }
            
            // Check for stall (safety)
            uint16_t sg = tmc2209_get_sg_result();
            if (sg < TMC_STALLGUARD_THRESH_OPERATE * 2 && sg != 0xFFFF && steps > 500) {
                ESP_LOGW(TAG, "Stall detected during unlock at step %d, revs=%.2f", steps, revs);
                break;
            }
        }
    }
    
    float final_revs = fabsf(as5600_get_total_revolutions());
    stepper_disable();
    
    xSemaphoreTake(state.mutex, portMAX_DELAY);
    state.motor_active = false;
    float final_angle = read_encoder_degrees();
    state.current_angle_deg = final_angle;
    
    // Determine final state
    float target_revs = UNLOCK_ROTATION_DEG / 360.0f;
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

