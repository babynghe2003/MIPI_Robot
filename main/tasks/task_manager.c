/* Task Manager Implementation */

#include "task_manager.h"
#include "app_config.h"
#include "esp_log.h"
#include "bldc_control.h"
#include <inttypes.h>

static const char *TAG = "task_manager";

esp_err_t task_manager_init(void) {
    ESP_LOGI(TAG, "Initializing task manager...");
    
    // Small delay to let system stabilize
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Create LED task if enabled
    if (MODULE_LED_ENABLED) {
        ESP_LOGI(TAG, "Creating LED task...");
        BaseType_t led_task_created = xTaskCreate(
            led_task, 
            "led_task", 
            TASK_STACK_SIZE_MEDIUM, 
            NULL, 
            TASK_PRIORITY_NORMAL, 
            NULL
        );
        if (led_task_created != pdPASS) {
            ESP_LOGE(TAG, "Failed to create LED task");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "LED task created successfully");
    }
    
    // Add delay between task creations
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Create servo task if enabled
    if (MODULE_SERVO_ENABLED) {
        ESP_LOGI(TAG, "Creating servo task...");
        BaseType_t servo_task_created = xTaskCreate(
            servo_task, 
            "servo_task", 
            TASK_STACK_SIZE_MEDIUM, 
            NULL, 
            TASK_PRIORITY_NORMAL, 
            NULL
        );
        if (servo_task_created != pdPASS) {
            ESP_LOGE(TAG, "Failed to create servo task");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Servo task created successfully");
    }
    
    // Create MPU9250 sensor task if enabled
    if (MODULE_SENSORS_ENABLED) {
        ESP_LOGI(TAG, "Creating MPU9250 sensor task...");
        BaseType_t mpu_task_created = xTaskCreate(
            mpu9250_task, 
            "mpu9250_task", 
            TASK_STACK_SIZE_LARGE,  // MPU9250 needs more stack for I2C operations
            NULL, 
            TASK_PRIORITY_NORMAL, 
            NULL
        );
        if (mpu_task_created != pdPASS) {
            ESP_LOGE(TAG, "Failed to create MPU9250 task");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "MPU9250 task created successfully");
        
        // Add delay after sensor initialization
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // Create BLDC task if enabled
    if (MODULE_BLDC_ENABLED) {
        ESP_LOGI(TAG, "Creating BLDC task...");
        BaseType_t bldc_task_created = xTaskCreate(
            bldc_task, 
            "bldc_task", 
            TASK_STACK_SIZE_LARGE,  // BLDC needs more stack for FOC calculations
            NULL, 
            TASK_PRIORITY_HIGH,     // Higher priority for real-time control
            NULL
        );
        if (bldc_task_created != pdPASS) {
            ESP_LOGE(TAG, "Failed to create BLDC task");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "BLDC task created successfully");
        
        // Add delay after BLDC initialization
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // Future: Add audio, communication tasks here
    /*
    if (MODULE_AUDIO_ENABLED) {
        // Create audio task
    }
    
    if (MODULE_WIFI_ENABLED) {
        // Create communication task
    }
    */
    
    ESP_LOGI(TAG, "All enabled tasks created successfully");
    return ESP_OK;
}

void task_manager_monitor(void) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        
        if (DEBUG_TASK_HEARTBEAT) {
            ESP_LOGI(TAG, "Task manager heartbeat - Free heap: %" PRIu32 " bytes", 
                     esp_get_free_heap_size());
            
            // Future: Add task-specific monitoring
            // - Check task stack high water marks
            // - Monitor task states
            // - Log system health metrics
        }
    }
}
