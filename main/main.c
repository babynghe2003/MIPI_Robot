/* MIPIRobot_NEO Main Application
   
   Modular Robot Control System with:
   - Servo control
   - LED indicators  
   - Expandable for sensors, audio, communication
*/

#include <stdio.h>
#include "app_config.h"
#include "task_manager.h"
#include "esp_log.h"
#include "pid_console.h"

static const char *TAG = "MIPIRobot_NEO";

void app_main(void)
{
    ESP_LOGI(TAG, "=== MIPIRobot_NEO Starting ===");
    ESP_LOGI(TAG, "Hardware: ESP32-S3");
    ESP_LOGI(TAG, "Enabled modules: %s%s%s%s%s", 
             MODULE_LED_ENABLED ? "LED " : "",
             MODULE_SERVO_ENABLED ? "SERVO " : "",
             MODULE_ROBOT_ENABLED? "ROBOT" : "",
             MODULE_AUDIO_ENABLED ? "AUDIO " : "",
             MODULE_WIFI_ENABLED ? "WIFI " : "");
    
    // Initialize task manager and start all enabled tasks
    esp_err_t ret = task_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize task manager");
        return;
    }
    // Start PID console over UART for live tuning
    pid_console_start();
    
    ESP_LOGI(TAG, "=== System Ready ===");
    
    // Enter monitoring loop
    task_manager_monitor();
}
