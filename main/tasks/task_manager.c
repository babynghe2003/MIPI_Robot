/* Task Manager Implementation */

#include "task_manager.h"
#include "app_config.h"
#include "esp_log.h"
#include "freertos/semphr.h"
#include <inttypes.h>

static const char *TAG = "task_manager";

// Global robot state và mutex bảo vệ
static robot_state_t current_robot_state = ROBOT_STATE_INIT;
static SemaphoreHandle_t state_mutex = NULL;

// LED RGB patterns cho từng trạng thái robot
static const led_rgb_pattern_t led_patterns[] = {
    [ROBOT_STATE_INIT]         = {0,   0,   100, 1000, 50, true},   // Xanh dương chậm
    [ROBOT_STATE_I2C_SETUP]    = {0,   0,   100, 200,  50, true},   // Xanh dương nhanh
    [ROBOT_STATE_SCAN_DEVICES] = {50,  0,   100, 300,  50, true},   // Tím
    [ROBOT_STATE_INIT_MPU9250] = {100, 100, 0,   400,  50, true},   // Vàng
    [ROBOT_STATE_CALIBRATING]  = {100, 100, 0,   100,  50, true},   // Vàng nhanh
    [ROBOT_STATE_INIT_MOTORS]  = {100, 50,  0,   250,  50, true},   // Cam
    [ROBOT_STATE_RUNNING]      = {0,   100, 0,   0,    100, false}, // Xanh lá ổn định
    [ROBOT_STATE_ERROR]        = {100, 0,   0,   150,  50, true},   // Đỏ nhanh
    [ROBOT_STATE_SENSOR_ERROR] = {100, 0,   0,   500,  50, true},   // Đỏ trung bình
    [ROBOT_STATE_MOTOR_ERROR]  = {100, 25,  0,   300,  50, true},   // Đỏ cam
};

void task_manager_set_robot_state(robot_state_t state)
{
    if (state_mutex != NULL) {
        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            robot_state_t old_state = current_robot_state;
            current_robot_state = state;
            xSemaphoreGive(state_mutex);
            
            // Log state change
            ESP_LOGI(TAG, "Robot state changed: %d -> %d", old_state, state);
        }
    } else {
        current_robot_state = state; // Fallback nếu mutex chưa tạo
    }
}

robot_state_t task_manager_get_robot_state(void)
{
    robot_state_t state = ROBOT_STATE_INIT;
    if (state_mutex != NULL) {
        if (xSemaphoreTake(state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            state = current_robot_state;
            xSemaphoreGive(state_mutex);
        }
    } else {
        state = current_robot_state;
    }
    return state;
}

led_rgb_pattern_t task_manager_get_led_rgb_pattern(void)
{
    robot_state_t state = task_manager_get_robot_state();
    if (state < sizeof(led_patterns) / sizeof(led_patterns[0])) {
        return led_patterns[state];
    }
    // Default error pattern
    return led_patterns[ROBOT_STATE_ERROR];
}

led_pattern_t task_manager_get_led_pattern(void)
{
    led_rgb_pattern_t rgb_pattern = task_manager_get_led_rgb_pattern();
    
    // Convert RGB to hex for legacy compatibility
    led_pattern_t legacy_pattern;
    legacy_pattern.color = (rgb_pattern.red << 16) | (rgb_pattern.green << 8) | rgb_pattern.blue;
    legacy_pattern.period_ms = rgb_pattern.period_ms;
    legacy_pattern.duty_cycle = rgb_pattern.duty_cycle;
    legacy_pattern.blink = rgb_pattern.blink;
    
    return legacy_pattern;
}

esp_err_t task_manager_init(void) {
    ESP_LOGI(TAG, "Initializing task manager...");
    
    // Tạo mutex cho robot state
    state_mutex = xSemaphoreCreateMutex();
    if (state_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create state mutex");
        return ESP_FAIL;
    }
    
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
        
        // Tạo parameters cho robot task với callback
        static robot_task_params_t robot_params = {
            .state_callback = task_manager_set_robot_state
        };
        
        BaseType_t robot_task_created = xTaskCreate(
            robot_task, 
            "robot_task", 
            TASK_STACK_SIZE_LARGE,  // MPU9250 needs more stack for I2C operations
            &robot_params,          // Truyền callback qua parameters
            TASK_PRIORITY_NORMAL, 
            NULL
        );
        if (robot_task_created != pdPASS) {
            ESP_LOGE(TAG, "Failed to create MPU9250 task");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "MPU9250 task created successfully");
        
        // Add delay after sensor initialization
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
