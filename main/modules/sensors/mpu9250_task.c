/* MPU9250 Sensor Task Implementation */

#include "mpu9250_control.h"
#include "app_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "mpu9250_task";

// Shared data structure for sensor readings
static struct {
    mpu9250_angles_t angles;
    mpu9250_accel_t accel;
    mpu9250_gyro_t gyro;
    mpu9250_mag_t mag;
    float temperature;
    bool data_ready;
} sensor_data = {0};

// Mutex for data synchronization
static SemaphoreHandle_t data_mutex = NULL;

// Task handles
static TaskHandle_t update_task_handle = NULL;
static TaskHandle_t display_task_handle = NULL;

/* MPU9250 Update Task - High frequency data acquisition (1ms interval) */
void mpu9250_update_task(void *pvParameters)
{
    ESP_LOGI(TAG, "MPU9250 update task started (1ms interval)");
    
    /* Initialize MPU9250 */
    esp_err_t ret = mpu9250_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU9250: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    
    /* Optional: Calibrate gyroscope */
    ESP_LOGI(TAG, "Starting gyroscope calibration...");
    ret = mpu9250_calibrate_gyro();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU9250 gyroscope calibration failed: %s", esp_err_to_name(ret));
    }
    
    ESP_LOGI(TAG, "Starting high-frequency MPU9250 data acquisition...");
    
    while (1) {
        /* Read sensor data at high frequency */
        ret = mpu9250_read_sensor();
        if (ret == ESP_OK) {
            /* Update shared data structure with mutex protection */
            if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                mpu9250_get_angles(&sensor_data.angles);
                mpu9250_get_accel(&sensor_data.accel);
                mpu9250_get_gyro(&sensor_data.gyro);
                mpu9250_get_mag(&sensor_data.mag);
                mpu9250_get_temperature(&sensor_data.temperature);
                sensor_data.data_ready = true;
                xSemaphoreGive(data_mutex);
            }
        }
        
        /* Wait 1ms before next reading (1000Hz update rate) */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/* MPU9250 Display Task - Low frequency data logging (100ms interval) */
void mpu9250_display_task(void *pvParameters)
{
    ESP_LOGI(TAG, "MPU9250 display task started (100ms interval)");
    
    // Local copies of sensor data
    mpu9250_angles_t angles;
    mpu9250_accel_t accel;
    mpu9250_gyro_t gyro;
    mpu9250_mag_t mag;
    float temperature;
    bool data_ready = false;
    
    while (1) {
        /* Copy data with mutex protection */
        if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (sensor_data.data_ready) {
                angles = sensor_data.angles;
                accel = sensor_data.accel;
                gyro = sensor_data.gyro;
                mag = sensor_data.mag;
                temperature = sensor_data.temperature;
                data_ready = true;
            }
            xSemaphoreGive(data_mutex);
        }
        
        /* Display data if ready */
        if (data_ready) {
            /* Log X, Y, Z angles and magnetic X, Y, Z */
            ESP_LOGI(TAG, "Angles -> X: %6.2f°, Y: %6.2f°, Z: %6.2f° | Mag -> X: %7.2f µT, Y: %7.2f µT, Z: %7.2f µT", 
                     angles.x, angles.y, angles.z, mag.x, mag.y, mag.z);
            
            /* Optional: Log additional data if debug is enabled */
            if (DEBUG_SENSOR_DATA) {
                ESP_LOGD(TAG, "Accel -> X: %6.2f m/s², Y: %6.2f m/s², Z: %6.2f m/s²", 
                         accel.x, accel.y, accel.z);
                
                ESP_LOGD(TAG, "Gyro  -> X: %6.3f rad/s, Y: %6.3f rad/s, Z: %6.3f rad/s", 
                         gyro.x, gyro.y, gyro.z);
                
                ESP_LOGD(TAG, "Temperature: %.1f°C", temperature);
            }
            
            data_ready = false;
        }
        
        /* Wait 100ms before next display */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* Original MPU9250 Sensor Task - Now creates two separate tasks */
void mpu9250_task(void *pvParameters)
{
    ESP_LOGI(TAG, "MPU9250 main task started - creating update and display tasks");
    
    /* Create mutex for data synchronization */
    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create data mutex");
        vTaskDelete(NULL);
        return;
    }
    
    /* Create high-frequency update task */
    BaseType_t ret = xTaskCreate(
        mpu9250_update_task,                 /* Task function */
        "mpu9250_update",                    /* Task name */
        4096,                                /* Stack size */
        NULL,                                /* Parameters */
        configMAX_PRIORITIES - 1,            /* High priority for real-time updates */
        &update_task_handle                  /* Task handle */
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create MPU9250 update task");
        vSemaphoreDelete(data_mutex);
        vTaskDelete(NULL);
        return;
    }
    
    /* Create low-frequency display task */
    ret = xTaskCreate(
        mpu9250_display_task,                /* Task function */
        "mpu9250_display",                   /* Task name */
        4096,                                /* Stack size */
        NULL,                                /* Parameters */
        tskIDLE_PRIORITY + 2,                /* Lower priority for display */
        &display_task_handle                 /* Task handle */
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create MPU9250 display task");
        /* Clean up update task and mutex */
        if (update_task_handle != NULL) {
            vTaskDelete(update_task_handle);
        }
        vSemaphoreDelete(data_mutex);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "MPU9250 tasks created successfully");
    
    /* Main task can now delete itself as the work is done by subtasks */
    vTaskDelete(NULL);
}
