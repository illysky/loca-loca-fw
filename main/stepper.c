#include "stepper.h"
#include "tmc2209.h"
#include "as5600.h"
#include "config.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"

static const char *TAG = "STEPPER";

// ============================================================================
// Internal State
// ============================================================================

static stepper_config_t stepper_config = {
    .max_speed_hz = MOTOR_MAX_SPEED_HZ,
    .accel_hz_per_s = MOTOR_ACCEL_HZ_PER_S,
    .homing_speed_hz = LOCK_SPEED_HZ,
    .travel_steps = LOCK_MAX_STEPS,
    .homing_timeout_ms = LOCK_TIMEOUT_MS,
    .backoff_steps = LOCK_BACKOFF_STEPS,
};

static struct {
    stepper_state_t state;
    lock_position_t position;
    int32_t current_step;
    int32_t target_step;
    int32_t steps_to_go;
    bool is_homed;
    bool stall_detected;
    uint32_t total_steps;
    stepper_dir_t direction;
    
    // Motion profile
    uint32_t current_speed_hz;
    uint32_t target_speed_hz;
    
    // Timer handle
    gptimer_handle_t timer;
    
    // Synchronization
    SemaphoreHandle_t mutex;
    SemaphoreHandle_t move_complete;
} stepper = {0};

// ============================================================================
// Timer ISR for Step Generation
// ============================================================================

static bool IRAM_ATTR step_timer_callback(gptimer_handle_t timer, 
                                           const gptimer_alarm_event_data_t *edata, 
                                           void *user_ctx) {
    BaseType_t high_task_woken = pdFALSE;
    
    // Run for both RUNNING and HOMING states
    if (stepper.steps_to_go > 0 && 
        (stepper.state == STEPPER_STATE_RUNNING || stepper.state == STEPPER_STATE_HOMING)) {
        
        // Check for stall - DIAG pin OR set flag for software check
        // (DIAG hardware detection seems broken, software check happens in main loop)
        if (gpio_get_level(PIN_TMC_DIAG) == 1) {
            stepper.state = STEPPER_STATE_STALLED;
            stepper.stall_detected = true;
            stepper.steps_to_go = 0;
            
            // Signal completion
            xSemaphoreGiveFromISR(stepper.move_complete, &high_task_woken);
        } else {
            // Generate step pulse
            gpio_set_level(PIN_TMC_STEP, 1);
            // Brief delay (step pulse must be >100ns for TMC2209)
            for (volatile int i = 0; i < 10; i++) {}
            gpio_set_level(PIN_TMC_STEP, 0);
            
            stepper.steps_to_go--;
            stepper.total_steps++;
            
            // Update position
            if (stepper.direction == STEPPER_DIR_CW) {
                stepper.current_step++;
            } else {
                stepper.current_step--;
            }
            
            // Check if done
            if (stepper.steps_to_go == 0) {
                stepper.state = STEPPER_STATE_IDLE;
                xSemaphoreGiveFromISR(stepper.move_complete, &high_task_woken);
            }
        }
    }
    
    return high_task_woken == pdTRUE;
}

// ============================================================================
// Timer Control
// ============================================================================

static esp_err_t timer_init(void) {
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,  // 1 MHz = 1us resolution
    };
    
    esp_err_t ret = gptimer_new_timer(&timer_config, &stepper.timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    gptimer_event_callbacks_t cbs = {
        .on_alarm = step_timer_callback,
    };
    gptimer_register_event_callbacks(stepper.timer, &cbs, NULL);
    
    gptimer_enable(stepper.timer);
    
    ESP_LOGI(TAG, "Step timer initialized");
    return ESP_OK;
}

static void timer_set_frequency(uint32_t freq_hz) {
    if (freq_hz == 0) return;
    
    uint64_t alarm_count = 1000000 / freq_hz;  // Period in microseconds
    
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = alarm_count,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    gptimer_set_alarm_action(stepper.timer, &alarm_config);
}

static void timer_start(void) {
    gptimer_set_raw_count(stepper.timer, 0);
    gptimer_start(stepper.timer);
}

static void timer_stop(void) {
    gptimer_stop(stepper.timer);
}

// ============================================================================
// Initialization
// ============================================================================

esp_err_t stepper_init(void) {
    ESP_LOGI(TAG, "Initializing stepper motor system");
    
    // Create synchronization primitives
    stepper.mutex = xSemaphoreCreateMutex();
    stepper.move_complete = xSemaphoreCreateBinary();
    
    if (stepper.mutex == NULL || stepper.move_complete == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphores");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize TMC2209 driver
    esp_err_t ret = tmc2209_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TMC2209 init failed");
        return ret;
    }
    
    // Configure TMC2209
    tmc2209_config_t tmc_config = {
        .run_current_ma = TMC_RUN_CURRENT_MA,
        .hold_current_ma = TMC_HOLD_CURRENT_MA,
        .microsteps = TMC_MICROSTEPS,
        .stallguard_thresh = TMC_STALLGUARD_THRESH,
        .stallguard_filter = TMC_STALLGUARD_FILTER,
        .stealthchop = TMC_STEALTHCHOP,
        .coolstep_enable = TMC_COOLSTEP_ENABLE,
        .coolstep_semin = TMC_COOLSTEP_SEMIN,
        .coolstep_semax = TMC_COOLSTEP_SEMAX,
        .coolstep_sedn = TMC_COOLSTEP_SEDN,
    };
    tmc2209_configure(&tmc_config);
    
    // Reset TMC2209 to ensure clean state (clears any power-up issues)
    tmc2209_reset();
    
    // Initialize step timer
    ret = timer_init();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Initialize state
    stepper.state = STEPPER_STATE_IDLE;
    stepper.position = LOCK_POSITION_UNKNOWN;
    stepper.current_step = 0;
    stepper.is_homed = false;
    stepper.stall_detected = false;
    
    ESP_LOGI(TAG, "Stepper motor initialized");
    ESP_LOGI(TAG, "  Max speed: %lu Hz", (unsigned long)stepper_config.max_speed_hz);
    ESP_LOGI(TAG, "  Travel steps: %ld", (long)stepper_config.travel_steps);
    
    return ESP_OK;
}

// ============================================================================
// Configuration
// ============================================================================

esp_err_t stepper_set_config(const stepper_config_t *config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(stepper.mutex, portMAX_DELAY);
    memcpy(&stepper_config, config, sizeof(stepper_config_t));
    xSemaphoreGive(stepper.mutex);
    
    ESP_LOGI(TAG, "Configuration updated");
    return ESP_OK;
}

esp_err_t stepper_get_status(stepper_status_t *status) {
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(stepper.mutex, portMAX_DELAY);
    status->state = stepper.state;
    status->position = stepper.position;
    status->current_step = stepper.current_step;
    status->target_step = stepper.target_step;
    status->is_homed = stepper.is_homed;
    status->stall_detected = stepper.stall_detected;
    status->steps_moved = stepper.total_steps;
    xSemaphoreGive(stepper.mutex);
    
    return ESP_OK;
}

// ============================================================================
// Enable/Disable
// ============================================================================

void stepper_enable(void) {
    tmc2209_enable();
    vTaskDelay(pdMS_TO_TICKS(10));  // Allow driver to stabilize
}

void stepper_disable(void) {
    timer_stop();
    
    // Set SGTHRS=0 before disabling to ensure DIAG goes low
    tmc2209_write_reg(0x40, 0);  // TMC_REG_SGTHRS = 0
    
    tmc2209_disable();
    stepper.state = STEPPER_STATE_IDLE;
}

// ============================================================================
// Motion Control
// ============================================================================

esp_err_t stepper_move_steps(int32_t steps, bool wait) {
    if (steps == 0) {
        return ESP_OK;
    }
    
    xSemaphoreTake(stepper.mutex, portMAX_DELAY);
    
    if (stepper.state == STEPPER_STATE_RUNNING) {
        xSemaphoreGive(stepper.mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Clear any previous stall
    stepper.stall_detected = false;
    
    // Set direction
    if (steps > 0) {
        stepper.direction = STEPPER_DIR_CW;
        gpio_set_level(PIN_TMC_DIR, 0);
    } else {
        stepper.direction = STEPPER_DIR_CCW;
        gpio_set_level(PIN_TMC_DIR, 1);
        steps = -steps;  // Make positive
    }
    
    stepper.steps_to_go = steps;
    stepper.target_step = stepper.current_step + 
                          (stepper.direction == STEPPER_DIR_CW ? steps : -steps);
    stepper.state = STEPPER_STATE_RUNNING;
    stepper.position = LOCK_POSITION_MOVING;
    
    // Clear completion semaphore
    xSemaphoreTake(stepper.move_complete, 0);
    
    // Configure and start timer
    timer_set_frequency(stepper_config.max_speed_hz);
    timer_start();
    
    xSemaphoreGive(stepper.mutex);
    
    ESP_LOGI(TAG, "Moving %ld steps %s", (long)steps, 
             stepper.direction == STEPPER_DIR_CW ? "CW" : "CCW");
    
    if (wait) {
        return stepper_wait_idle(30000);  // 30s timeout
    }
    
    return ESP_OK;
}

esp_err_t stepper_move_to(int32_t position, bool wait) {
    int32_t delta = position - stepper.current_step;
    return stepper_move_steps(delta, wait);
}

void stepper_stop(void) {
    timer_stop();
    stepper.steps_to_go = 0;
    stepper.state = STEPPER_STATE_IDLE;
    ESP_LOGI(TAG, "Motor stopped");
}

bool stepper_is_moving(void) {
    return stepper.state == STEPPER_STATE_RUNNING || 
           stepper.state == STEPPER_STATE_HOMING;
}

esp_err_t stepper_wait_idle(uint32_t timeout_ms) {
    if (xSemaphoreTake(stepper.move_complete, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        if (stepper.stall_detected) {
            ESP_LOGW(TAG, "Move stopped by stall");
            return ESP_ERR_TIMEOUT;
        }
        return ESP_OK;
    }
    
    stepper_stop();
    return ESP_ERR_TIMEOUT;
}

void stepper_set_zero(void) {
    xSemaphoreTake(stepper.mutex, portMAX_DELAY);
    stepper.current_step = 0;
    xSemaphoreGive(stepper.mutex);
    ESP_LOGI(TAG, "Position set to zero");
}

void stepper_clear_stall(void) {
    xSemaphoreTake(stepper.mutex, portMAX_DELAY);
    stepper.stall_detected = false;
    stepper.state = STEPPER_STATE_IDLE;
    xSemaphoreGive(stepper.mutex);
    
    // First set SGTHRS=0 and TCOOLTHRS=0 to disable StallGuard output entirely
    tmc2209_write_reg(0x40, 0);     // TMC_REG_SGTHRS = 0
    tmc2209_write_reg(0x14, 0);     // TMC_REG_TCOOLTHRS = 0 (disables StallGuard)
    
    // Toggle enable to fully reset driver state
    tmc2209_disable();
    vTaskDelay(pdMS_TO_TICKS(50));  // Longer delay
    
    // Clear GSTAT multiple times to be sure
    tmc2209_write_reg(0x01, 0x07);  // TMC_REG_GSTAT
    vTaskDelay(pdMS_TO_TICKS(10));
    tmc2209_write_reg(0x01, 0x07);  // Again
    
    // Write SGTHRS=0 again after EN toggle
    tmc2209_write_reg(0x40, 0);
    
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Check DIAG state - driver stays DISABLED so motor can spin free
    int diag = gpio_get_level(PIN_TMC_DIAG);
    
    ESP_LOGI("STEPPER", "Stall cleared: DIAG=%d, driver DISABLED (motor free)", diag);
    
    if (diag != 0) {
        ESP_LOGW("STEPPER", "WARNING: DIAG still high after clear!");
    }
}

// ============================================================================
// Homing
// ============================================================================

esp_err_t stepper_home(stepper_dir_t direction) {
    ESP_LOGI(TAG, "Homing in %s direction", 
             direction == STEPPER_DIR_CW ? "CW" : "CCW");
    
    stepper_enable();
    vTaskDelay(pdMS_TO_TICKS(100));  // Let driver stabilize
    
    // Set direction
    gpio_set_level(PIN_TMC_DIR, direction);
    stepper.direction = direction;
    stepper.state = STEPPER_STATE_HOMING;
    stepper.stall_detected = false;
    
    // Calculate maximum steps (use a large number)
    int32_t max_steps = stepper_config.travel_steps * 3;
    
    xSemaphoreTake(stepper.mutex, portMAX_DELAY);
    stepper.steps_to_go = max_steps;
    xSemaphoreGive(stepper.mutex);
    
    // Clear completion semaphore
    xSemaphoreTake(stepper.move_complete, 0);
    
    // Start at homing speed (slower for reliable stall detection)
    timer_set_frequency(stepper_config.homing_speed_hz);
    timer_start();
    
    // Wait for stall or timeout
    if (xSemaphoreTake(stepper.move_complete, 
                       pdMS_TO_TICKS(stepper_config.homing_timeout_ms)) == pdTRUE) {
        timer_stop();
        
        if (stepper.stall_detected) {
            ESP_LOGI(TAG, "Stall detected - home found");
            
            // Back off a bit
            vTaskDelay(pdMS_TO_TICKS(100));
            stepper_clear_stall();
            
            // Reverse direction and back off
            gpio_set_level(PIN_TMC_DIR, !direction);
            
            xSemaphoreTake(stepper.mutex, portMAX_DELAY);
            stepper.steps_to_go = stepper_config.backoff_steps;
            stepper.state = STEPPER_STATE_RUNNING;
            xSemaphoreGive(stepper.mutex);
            
            xSemaphoreTake(stepper.move_complete, 0);
            timer_start();
            
            stepper_wait_idle(5000);
            timer_stop();
            
            // Set this as zero position
            stepper_set_zero();
            stepper.is_homed = true;
            stepper.state = STEPPER_STATE_IDLE;
            
            return ESP_OK;
        }
    }
    
    timer_stop();
    stepper.state = STEPPER_STATE_ERROR;
    ESP_LOGE(TAG, "Homing failed - no stall detected");
    return ESP_ERR_TIMEOUT;
}

esp_err_t stepper_calibrate(void) {
    ESP_LOGI(TAG, "=== Running Lock Test Cycle ===");
    
    // Test: Lock then Unlock to verify mechanism
    ESP_LOGI(TAG, "Step 1: Testing LOCK...");
    esp_err_t ret = lock_engage();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Lock test failed");
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "Step 2: Testing UNLOCK...");
    ret = lock_disengage();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Unlock test failed");
        return ret;
    }
    
    ESP_LOGI(TAG, "=== Lock Test Complete ===");
    stepper.is_homed = true;  // Mark as tested
    
    return ESP_OK;
}

// ============================================================================
// Lock Functions - Drive until stall detected
// ============================================================================

// Internal: Drive motor in direction until stall or timeout
static esp_err_t drive_until_stall(stepper_dir_t direction, const char *action_name) {
    ESP_LOGI(TAG, "%s - driving %s until stall...", action_name,
             direction == STEPPER_DIR_CW ? "CW" : "CCW");
    
    // Check initial DIAG state
    ESP_LOGI(TAG, "DIAG pin initial state: %d", gpio_get_level(PIN_TMC_DIAG));
    
    // Clear any previous stall FIRST (this disables the motor)
    stepper_clear_stall();
    
    // NOW enable for this movement
    stepper_enable();
    vTaskDelay(pdMS_TO_TICKS(100));  // Let driver stabilize
    
    // Print TMC status before starting
    tmc2209_print_status();
    
    // Force re-write TCOOLTHRS to ensure StallGuard is active
    // StallGuard only works when TSTEP < TCOOLTHRS
    ESP_LOGI(TAG, "Setting TCOOLTHRS=0xFFFFF to enable StallGuard at all speeds");
    tmc2209_write_reg(0x14, 0xFFFFF);  // TMC_REG_TCOOLTHRS
    
    // Write SGTHRS from config
    ESP_LOGI(TAG, "Setting SGTHRS=%d (stall when SG < %d)", TMC_STALLGUARD_THRESH, TMC_STALLGUARD_THRESH * 2);
    tmc2209_write_reg(0x40, TMC_STALLGUARD_THRESH);  // TMC_REG_SGTHRS
    
    // Set direction
    gpio_set_level(PIN_TMC_DIR, direction);
    stepper.direction = direction;
    stepper.state = STEPPER_STATE_HOMING;  // Use homing state for stall detection
    stepper.position = LOCK_POSITION_MOVING;
    
    // Set max steps (safety limit)
    xSemaphoreTake(stepper.mutex, portMAX_DELAY);
    stepper.steps_to_go = stepper_config.travel_steps;
    xSemaphoreGive(stepper.mutex);
    
    // Clear completion semaphore
    xSemaphoreTake(stepper.move_complete, 0);
    
    // Start moving at lock speed
    ESP_LOGI(TAG, "Starting motor at %lu Hz, max %ld steps", 
             (unsigned long)stepper_config.homing_speed_hz,
             (long)stepper_config.travel_steps);
    timer_set_frequency(stepper_config.homing_speed_hz);
    timer_start();
    
    // Wait for stall or timeout, with periodic DIAG monitoring
    bool completed = false;
    uint32_t start_time = xTaskGetTickCount();
    uint32_t timeout_ticks = pdMS_TO_TICKS(stepper_config.homing_timeout_ms);
    uint32_t last_print = 0;
    
    while (!completed && (xTaskGetTickCount() - start_time) < timeout_ticks) {
        // Check every 100ms
        if (xSemaphoreTake(stepper.move_complete, pdMS_TO_TICKS(100)) == pdTRUE) {
            completed = true;
            break;
        }
        
        // Check SG_RESULT every 100ms for SOFTWARE stall detection
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - last_print > 100) {
            last_print = now;
            
            // Read SG_RESULT directly from dedicated register (0x41)
            uint16_t sg = tmc2209_get_sg_result();
            
            // SOFTWARE STALL DETECTION: Check if SG < threshold
            uint16_t sw_threshold = TMC_STALLGUARD_THRESH * 2;
            
            ESP_LOGI(TAG, "  [%lums] Steps:%ld SG=%d (thresh=%d) %s", 
                     (unsigned long)(now - (start_time * portTICK_PERIOD_MS)),
                     (long)stepper.steps_to_go, sg, sw_threshold,
                     (sg < sw_threshold) ? "<-- STALL DETECTED!" : "");
            
            // Software stall detection!
            if (sg < sw_threshold && sg != 0xFFFF) {
                ESP_LOGW(TAG, "*** SOFTWARE STALL DETECTED! SG=%d < %d ***", sg, sw_threshold);
                timer_stop();
                stepper.stall_detected = true;
                stepper.state = STEPPER_STATE_STALLED;
                xSemaphoreGive(stepper.move_complete);
                completed = true;
                break;
            }
        }
    }
    timer_stop();
    
    if (completed && stepper.stall_detected) {
        ESP_LOGI(TAG, "%s - stall detected, backing off", action_name);
        
        // Brief pause
        vTaskDelay(pdMS_TO_TICKS(100));
        stepper_clear_stall();
        
        // Back off slightly from the end stop
        gpio_set_level(PIN_TMC_DIR, !direction);  // Reverse direction
        
        xSemaphoreTake(stepper.mutex, portMAX_DELAY);
        stepper.steps_to_go = stepper_config.backoff_steps;
        stepper.state = STEPPER_STATE_RUNNING;
        xSemaphoreGive(stepper.mutex);
        
        xSemaphoreTake(stepper.move_complete, 0);
        timer_start();
        stepper_wait_idle(2000);
        timer_stop();
        
        stepper.state = STEPPER_STATE_IDLE;
        return ESP_OK;
    }
    
    // No stall detected - either timeout or completed all steps without stall
    stepper.state = STEPPER_STATE_IDLE;
    
    if (!stepper.stall_detected) {
        ESP_LOGW(TAG, "%s - no stall detected (may already be at position)", action_name);
        // This isn't necessarily an error - might already be at the end
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "%s - timeout", action_name);
    return ESP_ERR_TIMEOUT;
}

// Internal: Drive motor in direction until stall or timeout, with encoder tracking
static esp_err_t drive_until_stall_with_encoder(stepper_dir_t direction, const char *action_name) {
    ESP_LOGI(TAG, "%s - driving %s until stall (tracking revolutions)...", action_name,
             direction == STEPPER_DIR_CW ? "CW" : "CCW");
    
    // Check initial DIAG state
    ESP_LOGI(TAG, "DIAG pin initial state: %d", gpio_get_level(PIN_TMC_DIAG));
    
    // Clear any previous stall FIRST (this disables the motor)
    stepper_clear_stall();
    
    // NOW enable for this movement
    stepper_enable();
    vTaskDelay(pdMS_TO_TICKS(100));  // Let driver stabilize
    
    // Print TMC status before starting
    tmc2209_print_status();
    
    // Force re-write TCOOLTHRS to ensure StallGuard is active
    ESP_LOGI(TAG, "Setting TCOOLTHRS=0xFFFFF to enable StallGuard at all speeds");
    tmc2209_write_reg(0x14, 0xFFFFF);  // TMC_REG_TCOOLTHRS
    
    // Write SGTHRS from config
    ESP_LOGI(TAG, "Setting SGTHRS=%d (stall when SG < %d)", TMC_STALLGUARD_THRESH, TMC_STALLGUARD_THRESH * 2);
    tmc2209_write_reg(0x40, TMC_STALLGUARD_THRESH);  // TMC_REG_SGTHRS
    
    // Set direction
    gpio_set_level(PIN_TMC_DIR, direction);
    stepper.direction = direction;
    stepper.state = STEPPER_STATE_HOMING;
    stepper.position = LOCK_POSITION_MOVING;
    
    // Set max steps (safety limit)
    xSemaphoreTake(stepper.mutex, portMAX_DELAY);
    stepper.steps_to_go = stepper_config.travel_steps;
    xSemaphoreGive(stepper.mutex);
    
    // Clear completion semaphore
    xSemaphoreTake(stepper.move_complete, 0);
    
    // Start moving at lock speed
    ESP_LOGI(TAG, "Starting motor at %lu Hz, max %ld steps", 
             (unsigned long)stepper_config.homing_speed_hz,
             (long)stepper_config.travel_steps);
    timer_set_frequency(stepper_config.homing_speed_hz);
    timer_start();
    
    // Wait for stall or timeout, with periodic encoder updates
    bool completed = false;
    uint32_t start_time = xTaskGetTickCount();
    uint32_t timeout_ticks = pdMS_TO_TICKS(stepper_config.homing_timeout_ms);
    uint32_t last_print = 0;
    
    while (!completed && (xTaskGetTickCount() - start_time) < timeout_ticks) {
        // Check every 50ms for faster encoder updates
        if (xSemaphoreTake(stepper.move_complete, pdMS_TO_TICKS(50)) == pdTRUE) {
            completed = true;
            break;
        }
        
        // Update encoder tracking
        as5600_update();
        
        // Check SG_RESULT every 100ms for SOFTWARE stall detection
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - last_print > 100) {
            last_print = now;
            
            // Read SG_RESULT directly from dedicated register (0x41)
            uint16_t sg = tmc2209_get_sg_result();
            
            // Get current revolution count
            float revs = as5600_get_total_revolutions();
            
            // SOFTWARE STALL DETECTION: Check if SG < threshold
            uint16_t sw_threshold = TMC_STALLGUARD_THRESH * 2;
            
            ESP_LOGI(TAG, "  [%lums] Steps:%ld SG=%d Revs=%.2f %s", 
                     (unsigned long)(now - (start_time * portTICK_PERIOD_MS)),
                     (long)stepper.steps_to_go, sg, revs,
                     (sg < sw_threshold) ? "<-- STALL!" : "");
            
            // Software stall detection!
            if (sg < sw_threshold && sg != 0xFFFF) {
                ESP_LOGW(TAG, "*** SOFTWARE STALL DETECTED! SG=%d < %d ***", sg, sw_threshold);
                timer_stop();
                stepper.stall_detected = true;
                stepper.state = STEPPER_STATE_STALLED;
                xSemaphoreGive(stepper.move_complete);
                completed = true;
                break;
            }
        }
    }
    timer_stop();
    
    // Final encoder update
    as5600_update();
    
    if (completed && stepper.stall_detected) {
        ESP_LOGI(TAG, "%s - stall detected, backing off", action_name);
        
        // Brief pause
        vTaskDelay(pdMS_TO_TICKS(100));
        stepper_clear_stall();
        
        // Back off slightly from the end stop
        gpio_set_level(PIN_TMC_DIR, !direction);  // Reverse direction
        
        xSemaphoreTake(stepper.mutex, portMAX_DELAY);
        stepper.steps_to_go = stepper_config.backoff_steps;
        stepper.state = STEPPER_STATE_RUNNING;
        xSemaphoreGive(stepper.mutex);
        
        xSemaphoreTake(stepper.move_complete, 0);
        timer_start();
        stepper_wait_idle(2000);
        timer_stop();
        
        // Update encoder after backoff
        as5600_update();
        
        stepper.state = STEPPER_STATE_IDLE;
        return ESP_OK;
    }
    
    // No stall detected - either timeout or completed all steps without stall
    stepper.state = STEPPER_STATE_IDLE;
    
    if (!stepper.stall_detected) {
        ESP_LOGW(TAG, "%s - no stall detected (may already be at position)", action_name);
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "%s - timeout", action_name);
    return ESP_ERR_TIMEOUT;
}

esp_err_t lock_engage(void) {
    ESP_LOGI(TAG, "🔒 Engaging lock...");
    
    esp_err_t ret = drive_until_stall(STEPPER_DIR_CCW, "LOCK");  // Swapped direction
    
    if (ret == ESP_OK) {
        stepper.position = LOCK_POSITION_LOCKED;
        ESP_LOGI(TAG, "✅ Lock engaged");
    } else {
        stepper.position = LOCK_POSITION_UNKNOWN;
        ESP_LOGE(TAG, "❌ Failed to engage lock");
    }
    
    return ret;
}

esp_err_t lock_disengage(void) {
    ESP_LOGI(TAG, "🔓 Disengaging lock...");
    
    // Reset AS5600 revolution counter before starting unlock
    as5600_reset_revolutions();
    
    esp_err_t ret = drive_until_stall_with_encoder(STEPPER_DIR_CW, "UNLOCK");  // Swapped direction
    
    // Get final revolution count
    float total_revs = as5600_get_total_revolutions();
    
    if (ret == ESP_OK) {
        stepper.position = LOCK_POSITION_UNLOCKED;
        ESP_LOGI(TAG, "✅ Lock disengaged");
        ESP_LOGI(TAG, "╔═══════════════════════════════════════╗");
        ESP_LOGI(TAG, "║  🔄 UNLOCK REVOLUTION COUNT: %.2f    ║", total_revs);
        ESP_LOGI(TAG, "╚═══════════════════════════════════════╝");
    } else {
        stepper.position = LOCK_POSITION_UNKNOWN;
        ESP_LOGE(TAG, "❌ Failed to disengage lock");
        ESP_LOGI(TAG, "  Revolutions before failure: %.2f", total_revs);
    }
    
    return ret;
}

lock_position_t lock_get_position(void) {
    return stepper.position;
}

bool lock_is_calibrated(void) {
    return stepper.is_homed;
}

