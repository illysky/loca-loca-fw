#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"

// ============================================================================
// ESP32-C3 Mini GPIO Pin Configuration for Lock Project
// ============================================================================
// 
// Available GPIOs on ESP32-C3 Mini:
//   GPIO0  - Can be used (strapping pin - has pull-up)
//   GPIO1  - Can be used (strapping pin)
//   GPIO2  - Can be used (strapping pin - has pull-down)
//   GPIO3  - Can be used
//   GPIO4  - Can be used
//   GPIO5  - Can be used
//   GPIO6  - Can be used
//   GPIO7  - Can be used
//   GPIO8  - Built-in LED (active high on most boards)
//   GPIO9  - Boot button (strapping pin - has pull-up)
//   GPIO10 - Can be used
//   GPIO18 - USB D- (avoid if using USB)
//   GPIO19 - USB D+ (avoid if using USB)
//   GPIO20 - UART0 RX (avoid if using USB/UART)
//   GPIO21 - UART0 TX (avoid if using USB/UART)
//
// TMC2209 Adafruit Breakout Pinout:
//   VM   - Motor voltage (external supply)
//   GND  - Ground
//   VIO  - Logic reference (3.3V from ESP32)
//   STEP - Step pulse input
//   DIR  - Direction input
//   EN   - Enable (active LOW)
//   M0   - Microstep select 0 (tie to GND or VIO)
//   M1   - Microstep select 1 (tie to GND or VIO)
//   DIAG - Diagnostic output (StallGuard indicator)
//   TX   - UART TX (to ESP RX)
//   RX   - UART RX (from ESP TX via 1k resistor)
// ============================================================================

// -----------------------------------------------------------------------------
// TMC2209 Stepper Driver Pin Configuration
// -----------------------------------------------------------------------------
        #define PIN_TMC_STEP            GPIO_NUM_4      // Step pulse output
        #define PIN_TMC_DIR             GPIO_NUM_3      // Direction output
#define PIN_TMC_EN              GPIO_NUM_9      // Enable output (active LOW)
#define PIN_TMC_DIAG            GPIO_NUM_20     // StallGuard/DIAG input
#define PIN_TMC_INDEX           GPIO_NUM_21     // INDEX output from TMC2209

// Microstep select pins (directly control MS1/MS2 on breakout)
// MS1  MS2  Microsteps
//  L    L   8
//  H    L   32 (with interpolation to 256)
//  L    H   64 (with interpolation to 256)
//  H    H   16 (with interpolation to 256)
#define PIN_TMC_MS1             GPIO_NUM_2      // Microstep select 1
#define PIN_TMC_MS2             GPIO_NUM_1      // Microstep select 2

// TMC2209 UART pins (two-wire half-duplex)
// Wiring: TX --[2.2k resistor]--+-- TMC2209 UART pin
//         RX -------------------+
#define PIN_TMC_UART_TX         GPIO_NUM_10     // UART TX (via resistor)
#define PIN_TMC_UART_RX         GPIO_NUM_5      // UART RX (direct)

// -----------------------------------------------------------------------------
// AS5600 Rotary Encoder (I2C)
// -----------------------------------------------------------------------------
#define PIN_AS5600_SDA          GPIO_NUM_6      // I2C SDA
#define PIN_AS5600_SCL          GPIO_NUM_7      // I2C SCL
#define AS5600_I2C_PORT         I2C_NUM_0       // I2C port number
#define AS5600_I2C_FREQ_HZ      400000          // I2C frequency (400kHz fast mode)

// -----------------------------------------------------------------------------
// Status LED
// -----------------------------------------------------------------------------
#define PIN_LED                 GPIO_NUM_8      // Built-in LED on Super Mini
#define LED_ACTIVE_LOW          true            // ESP32-C3 Super Mini LED is active LOW

// -----------------------------------------------------------------------------
// TMC2209 UART Configuration
// -----------------------------------------------------------------------------
#define TMC_UART_NUM            UART_NUM_1      // Use UART1 for TMC2209
#define TMC_UART_BAUD           115200          // Default TMC2209 baud rate
#define TMC_SLAVE_ADDR          3               // MS1=HIGH, MS2=HIGH = address 3

// -----------------------------------------------------------------------------
// TMC2209 Driver Settings
// -----------------------------------------------------------------------------

// Motor current settings (adjust for your motor!)
// RMS current in mA - typical NEMA17: 500-1500mA
#define TMC_RUN_CURRENT_MA      500             // Running current (mA)
#define TMC_HOLD_CURRENT_MA     100             // Holding current (mA)

// Microstepping (1, 2, 4, 8, 16, 32, 64, 128, 256)
#define TMC_MICROSTEPS          16              // Microsteps per full step

// StallGuard threshold (0-255)
// Higher = more sensitive (stall triggers at higher SG values)
// Lower = less sensitive (only triggers when SG is very low)
#define TMC_STALLGUARD_THRESH_CALIBRATE  100   // Calibration threshold (more sensitive)
#define TMC_STALLGUARD_THRESH_OPERATE    10    // Normal operation threshold (less sensitive)
#define TMC_STALLGUARD_THRESH   TMC_STALLGUARD_THRESH_OPERATE  // Default for legacy code

// StallGuard filter (true = enable 4-step filter)
#define TMC_STALLGUARD_FILTER   true            // Enable SG filter for stability

// Coolstep settings (current scaling based on load) - REQUIRED for StallGuard DIAG output!
#define TMC_COOLSTEP_ENABLE     true            // Enable CoolStep feature
#define TMC_COOLSTEP_SEMIN      5               // Lower threshold (from working example)
#define TMC_COOLSTEP_SEMAX      2               // Upper threshold (from working example)
#define TMC_COOLSTEP_SEDN       1               // Current decrement speed (0b01)

// SpreadCycle vs StealthChop
// NOTE: Working example says StealthChop mode + CoolStep is required for StallGuard!
#define TMC_STEALTHCHOP         true            // StealthChop mode (with CoolStep for StallGuard)

// -----------------------------------------------------------------------------
// Stepper Motor Settings
// -----------------------------------------------------------------------------

// Motor steps per revolution (typically 200 for 1.8° motors)
#define MOTOR_STEPS_PER_REV     200

// Speed settings
#define MOTOR_MAX_SPEED_HZ      6000            // Maximum step frequency (Hz)
#define MOTOR_ACCEL_HZ_PER_S    4000            // Acceleration (Hz/s)

// Lock mechanism settings
// Gear ratio: 40 teeth (thumb) / 16 teeth (motor) = 2.5:1
// Thumb needs ~1.2 rotations = stepper needs ~3 rotations
// Maximum steps before timeout (safety limit) - set high since we use stall detection
#define LOCK_MAX_STEPS          (MOTOR_STEPS_PER_REV * TMC_MICROSTEPS * 4)  // 4 full revs max

// Stall-based movement settings
#define LOCK_SPEED_HZ           4000            // Speed for lock/unlock (Hz) - ~150 RPM with 16 microsteps
#define CALIBRATE_SPEED_HZ      2000            // Speed for calibration (Hz) - slower for reliable stall detection
#define LOCK_BACKOFF_STEPS      50              // Steps to back off after stall detected
#define LOCK_TIMEOUT_MS         10000           // Maximum time for lock/unlock operation

// -----------------------------------------------------------------------------
// Lock State Tracking Settings
// -----------------------------------------------------------------------------
#define LOCK_ANGLE_TOLERANCE_DEG    5.0f        // Tolerance for considering position "locked" (degrees)
#define UNLOCK_ROTATION_DEG         310.0f      // Degrees to rotate for unlock
#define ENCODER_POLL_INTERVAL_MS    100         // How often to poll encoder (ms)
#define LOCK_BACKOFF_DEG            5.0f        // Degrees to back off after calibration stall

// Encoder-based stall detection sensitivity
// Higher values = LESS sensitive (harder to trigger stall)
#define STALL_MIN_MOVEMENT_DEG      1.0f        // Min movement per check to count as "moving" (lower = less sensitive)
#define STALL_COUNT_THRESHOLD       5           // Consecutive no-movement checks before stall (higher = less sensitive)

// -----------------------------------------------------------------------------
// WiFi Configuration
// -----------------------------------------------------------------------------
#define WIFI_SSID               "The Shack IOT"
#define WIFI_PASSWORD           "H3lloworld"

// -----------------------------------------------------------------------------
// Debug Settings
// -----------------------------------------------------------------------------
#define DEBUG_TMC_REGISTERS     false           // Print TMC register values
#define DEBUG_STEPPER_STATUS    false           // Print stepper status periodically

#endif // CONFIG_H

