/* Servo Control Wrapper for ESP-IDF Servo Component */

#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "iot_servo.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "app_config.h"

// Servo configuration constants
#define SERVO_CHANNEL               LEDC_CHANNEL_0
#define SERVO_TIMER                 LEDC_TIMER_0

/**
 * @brief Initialize servo with default configuration
 */
esp_err_t servo_init(void);

/**
 * @brief Set servo angle (0-180 degrees)
 */
esp_err_t servo_set_angle(float angle);

/**
 * @brief Deinitialize servo
 */
esp_err_t servo_deinit(void);

#endif // SERVO_CONTROL_H
