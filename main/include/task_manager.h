/* Task Manager Header */

#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

// Task function prototypes
void led_task(void *pvParameters);
void servo_task(void *pvParameters);
void mpu9250_task(void *pvParameters);

// Future task prototypes
void audio_task(void *pvParameters);
void communication_task(void *pvParameters);

/**
 * @brief Initialize and start all enabled tasks
 */
esp_err_t task_manager_init(void);

/**
 * @brief Monitor all running tasks
 */
void task_manager_monitor(void);

#endif // TASK_MANAGER_H
