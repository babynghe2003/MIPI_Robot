// C++ includes
#include <cstdio>
#include <cstring>
#include <string>

// ESP-IDF and project includes
#include "mpu9250_control.h"
#include "app_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Custom sensor includes
extern "C" {
#include "tca9548a.h"
#include "as5600.h"
#include "i2c_bus.h"
#include "driver/i2c.h"
}

static const char *TAG = "robot_task";

// Shared I2C bus handle for all sensors
static i2c_bus_handle_t shared_i2c_bus = nullptr;
static tca9548a_handle_t* mux_handle = nullptr;

// AS5600 sensor instances
static as5600_t* as5600_left = nullptr;   // Channel 0 
static as5600_t* as5600_right = nullptr;  // Channel 1

// Sensor data structure
static struct {
    // MPU9250 data
    mpu9250_angles_t angles;
    mpu9250_accel_t accel;
    mpu9250_gyro_t gyro;
    mpu9250_mag_t mag;
    float temperature;
    
    // AS5600 data
    float as5600_left_angle;
    float as5600_right_angle;
    
    bool data_ready;
} sensor_data = {};

static SemaphoreHandle_t data_mutex = nullptr;
static TaskHandle_t update_task_handle = nullptr;
static TaskHandle_t display_task_handle = nullptr;

extern "C" void robot_update_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Robot update task started (1ms interval)");

    // Create shared I2C bus for all sensors
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ,
        },
        .clk_flags = 0,
    };
    
    shared_i2c_bus = i2c_bus_create(I2C_NUM_0, &i2c_conf);
    if (!shared_i2c_bus) {
        ESP_LOGE(TAG, "Failed to create shared I2C bus");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Shared I2C bus created successfully");

    // Initialize TCA9548A multiplexer
    mux_handle = tca9548a_init(shared_i2c_bus, TCA9548A_DEFAULT_ADDR);
    if (!mux_handle) {
        ESP_LOGE(TAG, "Failed to initialize TCA9548A");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "TCA9548A multiplexer initialized");

    // Debug: Scan all channels to see what devices are available
    ESP_LOGI(TAG, "Scanning all TCA9548A channels for devices...");
    for (int ch = 0; ch < 8; ch++) {
        ESP_LOGI(TAG, "Scanning channel %d:", ch);
        esp_err_t scan_ret = tca9548a_select_channel(mux_handle, ch);
        if (scan_ret == ESP_OK) {
            // Simple I2C scan on this channel
            for (int addr = 0x08; addr < 0x78; addr++) {
                i2c_cmd_handle_t cmd = i2c_cmd_link_create();
                i2c_master_start(cmd);
                i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
                i2c_master_stop(cmd);
                esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(10));
                i2c_cmd_link_delete(cmd);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "  Device found at address 0x%02X", addr);
                }
            }
        } else {
            ESP_LOGE(TAG, "  Failed to select channel %d", ch);
        }
    }

    // Initialize MPU9250 on channel 0 with shared bus
    ESP_LOGI(TAG, "Initializing MPU9250 on channel 0...");
    esp_err_t ret = tca9548a_select_channel(mux_handle, TCA9548A_CHANNEL_1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select MPU9250 channel: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    
    ret = mpu9250_init_with_bus(shared_i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU9250: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "MPU9250 initialized on channel 0");

    // Calibrate MPU9250 gyroscope
    ESP_LOGI(TAG, "Starting gyroscope calibration...");
    ret = mpu9250_calibrate_gyro();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU9250 gyroscope calibration failed: %s", esp_err_to_name(ret));
    }

    // Initialize AS5600 left sensor on channel 0 (same as MPU9250)
    ret = tca9548a_select_channel(mux_handle, TCA9548A_CHANNEL_0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select AS5600 left channel: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    
    as5600_left = as5600_init(shared_i2c_bus, "LEFT");
    if (!as5600_left) {
        ESP_LOGE(TAG, "Failed to initialize AS5600 left sensor");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "AS5600 left sensor initialized on channel 0");

    // Initialize AS5600 right sensor on channel 1  
    ret = tca9548a_select_channel(mux_handle, TCA9548A_CHANNEL_1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select AS5600 right channel: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    
    as5600_right = as5600_init(shared_i2c_bus, "RIGHT");
    if (!as5600_right) {
        ESP_LOGE(TAG, "Failed to initialize AS5600 right sensor");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "AS5600 right sensor initialized on channel 1");

    ESP_LOGI(TAG, "Starting high-frequency sensor data acquisition...");
    
    while (1) {
        // Read MPU9250 data (channel 0)
        ret = tca9548a_select_channel(mux_handle, TCA9548A_CHANNEL_1);
        if (ret == ESP_OK) {
            ret = mpu9250_read_sensor();
        }
        
        if (ret == ESP_OK) {
            ret = as5600_read(as5600_right);
        }
        
        // Read AS5600 right sensor data (channel 1)
        ret = tca9548a_select_channel(mux_handle, TCA9548A_CHANNEL_0);
        // Read AS5600 left sensor data (channel 0)
        if (ret == ESP_OK) {
            ret = as5600_read(as5600_left);
        }
        // Update shared data structure with mutex protection
        if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            // MPU9250 data
            mpu9250_get_angles(&sensor_data.angles);
            mpu9250_get_accel(&sensor_data.accel);
            mpu9250_get_gyro(&sensor_data.gyro);
            mpu9250_get_mag(&sensor_data.mag);
            mpu9250_get_temperature(&sensor_data.temperature);
            
            // AS5600 data
            sensor_data.as5600_left_angle = as5600_get_angle_deg(as5600_left);
            sensor_data.as5600_right_angle = as5600_get_angle_deg(as5600_right);
            
            sensor_data.data_ready = true;
            xSemaphoreGive(data_mutex);
        }
        
        /* Wait 1ms before next reading (1000Hz update rate) */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

extern "C" void mpu9250_display_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Sensor display task started (100ms interval)");
    
    // Local copies of sensor data
    mpu9250_angles_t angles;
    mpu9250_accel_t accel;
    mpu9250_gyro_t gyro;
    mpu9250_mag_t mag;
    float temperature;
    float left_angle, right_angle;
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
                left_angle = sensor_data.as5600_left_angle;
                right_angle = sensor_data.as5600_right_angle;
                data_ready = true;
            }
            xSemaphoreGive(data_mutex);
        }
        
        /* Display all 3 sensors in one log line */
        if (data_ready) {
            ESP_LOGI(TAG, "MPU9250[X:%6.2f° Y:%6.2f° Z:%6.2f°] AS5600-L[%6.2f°] AS5600-R[%6.2f°] Temp[%.1f°C]", 
                     angles.x, angles.y, angles.z, 
                     left_angle, right_angle, 
                     temperature);
            
            if (DEBUG_SENSOR_DATA) {
                ESP_LOGD(TAG, "Accel -> X: %6.2f m/s², Y: %6.2f m/s², Z: %6.2f m/s²", 
                         accel.x, accel.y, accel.z);
                ESP_LOGD(TAG, "Gyro  -> X: %6.3f rad/s, Y: %6.3f rad/s, Z: %6.3f rad/s", 
                         gyro.x, gyro.y, gyro.z);
                ESP_LOGD(TAG, "Mag   -> X: %7.2f µT, Y: %7.2f µT, Z: %7.2f µT", 
                         mag.x, mag.y, mag.z);
            }
            data_ready = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

extern "C" void robot_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Robot main task started - creating update and display tasks");
    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == nullptr) {
        ESP_LOGE(TAG, "Failed to create data mutex");
        vTaskDelete(NULL);
        return;
    }
    BaseType_t ret = xTaskCreate(
        robot_update_task,
        "robot_update",
        4096,
        NULL,
        configMAX_PRIORITIES - 1,
        &update_task_handle
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Robot update task");
        vSemaphoreDelete(data_mutex);
        vTaskDelete(NULL);
        return;
    }
    ret = xTaskCreate(
        mpu9250_display_task,
        "mpu9250_display",
        4096,
        NULL,
        tskIDLE_PRIORITY + 2,
        &display_task_handle
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create MPU9250 display task");
        if (update_task_handle != nullptr) {
            vTaskDelete(update_task_handle);
        }
        vSemaphoreDelete(data_mutex);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Robot tasks created successfully");
    vTaskDelete(NULL);
}
