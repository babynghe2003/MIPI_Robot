/* Servo Control Wrapper for ESP-IDF Servo Component (2 servos) */

#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "iot_servo.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "app_config.h"

// Servo configuration constants
// Two servos on GPIO 11 and GPIO 4
#define SERVO_TIMER                 LEDC_TIMER_0
#define SERVO_LED_SPEED_MODE        LEDC_LOW_SPEED_MODE

// Index within iot_servo driver (0 and 1)
#define SERVO_INDEX_1               0
#define SERVO_INDEX_2               1

/**
 * @brief Initialize servo with default configuration
 */
esp_err_t servo_init(void);

/**
 * @brief Set servo angle (0-180 degrees) for servo index (0 or 1)
 */
esp_err_t servo_set_angle_idx(int idx, float angle);

/**
 * @brief Set desired target angle; actual move can be handled smoothly by a task.
 */
esp_err_t servo_set_target_idx(int idx, float angle);

/**
 * @brief Get the last commanded hardware angle (current).
 */
float servo_get_current_idx(int idx);

/**
 * @brief Get the desired target angle.
 */
float servo_get_target_idx(int idx);

/**
 * @brief Legacy helper: set angle for servo 0 (kept for backward-compat)
 */
static inline esp_err_t servo_set_angle(float angle) {
	return servo_set_angle_idx(SERVO_INDEX_1, angle);
}

/**
 * @brief Deinitialize servo
 */
esp_err_t servo_deinit(void);

#endif // SERVO_CONTROL_H
