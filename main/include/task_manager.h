/* Task Manager Header */

#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

// Robot task status definitions
typedef enum {
    ROBOT_STATE_INIT,              // Khởi tạo - LED chậm xanh dương
    ROBOT_STATE_I2C_SETUP,         // Thiết lập I2C - LED chớp nhanh xanh dương  
    ROBOT_STATE_SCAN_DEVICES,      // Quét thiết bị - LED chớp tím
    ROBOT_STATE_INIT_MPU9250,      // Khởi tạo MPU9250 - LED chớp vàng
    ROBOT_STATE_CALIBRATING,       // Hiệu chuẩn gyro - LED chớp nhanh vàng
    ROBOT_STATE_INIT_MOTORS,       // Khởi tạo motor - LED chớp cam
    ROBOT_STATE_RUNNING,           // Chạy bình thường - LED xanh lá ổn định
    ROBOT_STATE_ERROR,             // Lỗi - LED đỏ chớp nhanh
    ROBOT_STATE_SENSOR_ERROR,      // Lỗi sensor - LED đỏ + vàng luân phiên
    ROBOT_STATE_MOTOR_ERROR        // Lỗi motor - LED đỏ + cam luân phiên
} robot_state_t;

// LED pattern definitions (legacy)
typedef struct {
    uint32_t color;      // RGB color (0xRRGGBB)
    uint32_t period_ms;  // Thời gian chu kỳ (ms)
    uint8_t duty_cycle;  // Độ rộng xung (0-100%)
    bool blink;          // true = chớp, false = sáng liên tục
} led_pattern_t;

// LED RGB pattern definitions (new)
typedef struct {
    uint8_t red;
    uint8_t green; 
    uint8_t blue;
    uint32_t period_ms;
    uint8_t duty_cycle;
    bool blink;
} led_rgb_pattern_t;

// Task function prototypes
void led_task(void *pvParameters);
void servo_task(void *pvParameters);

// Robot state callback function type
typedef void (*robot_state_callback_t)(robot_state_t state);

// Parameter structure for robot task
typedef struct {
    robot_state_callback_t state_callback;
} robot_task_params_t;

// Robot task với callback parameter
void robot_task(void *pvParameters);
/**
 * @brief Initialize and start all enabled tasks
 */
esp_err_t task_manager_init(void);

/**
 * @brief Monitor all running tasks
 */
void task_manager_monitor(void);

/**
 * @brief Set robot task status để cập nhật LED pattern
 * @param state Trạng thái hiện tại của robot task
 */
void task_manager_set_robot_state(robot_state_t state);

/**
 * @brief Get current robot state
 * @return Current robot state
 */
robot_state_t task_manager_get_robot_state(void);

/**
 * @brief Get LED pattern for current robot state (legacy)
 * @return LED pattern structure
 */
led_pattern_t task_manager_get_led_pattern(void);

/**
 * @brief Get LED RGB pattern for current robot state
 * @return LED RGB pattern structure
 */
led_rgb_pattern_t task_manager_get_led_rgb_pattern(void);

#endif // TASK_MANAGER_H
