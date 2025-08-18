/* AS5600 Sensor Implementation */

#include "as5600.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

static const char *TAG = "AS5600";

as5600_t* as5600_init(i2c_bus_handle_t bus_handle, const char* name) {
    if (!bus_handle || !name) {
        ESP_LOGE(TAG, "Invalid parameters for AS5600 init");
        return NULL;
    }

    // Allocate sensor structure
    as5600_t* sensor = (as5600_t*)malloc(sizeof(as5600_t));
    if (!sensor) {
        ESP_LOGE(TAG, "Failed to allocate memory for AS5600 sensor");
        return NULL;
    }

    // Initialize structure
    memset(sensor, 0, sizeof(as5600_t));
    sensor->bus_handle = bus_handle;
    sensor->name = name;

    // Create device handle for AS5600
    sensor->device_handle = i2c_bus_device_create(bus_handle, AS5600_I2C_ADDR, 0);
    if (!sensor->device_handle) {
        ESP_LOGE(TAG, "Failed to create AS5600 device handle for %s", name);
        free(sensor);
        return NULL;
    }

    // Test connection
    esp_err_t ret = as5600_test_connection(sensor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AS5600 %s connection test failed", name);
        i2c_bus_device_delete(&sensor->device_handle);
        free(sensor);
        return NULL;
    }

    sensor->initialized = true;
    ESP_LOGI(TAG, "AS5600 %s initialized successfully", name);
    
    return sensor;
}

esp_err_t as5600_read(as5600_t* sensor) {
    if (!sensor || !sensor->initialized) {
        ESP_LOGE(TAG, "AS5600 sensor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Read raw angle from register 0x0C (2 bytes)
    uint8_t data[2];
    esp_err_t ret = i2c_bus_read_bytes(sensor->device_handle, AS5600_REG_RAW_ANGLE, 2, data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read AS5600 %s angle: %s", sensor->name, esp_err_to_name(ret));
        return ret;
    }

    // Combine bytes (big endian)
    sensor->raw_angle = (uint16_t)((data[0] << 8) | data[1]);
    
    // Mask to 12 bits (0-4095) and convert to radians
    sensor->raw_angle &= 0x0FFF;
    sensor->angle_rad = (float)sensor->raw_angle * 2.0f * M_PI / 4096.0f;

    return ESP_OK;
}

float as5600_get_angle_rad(as5600_t* sensor) {
    if (!sensor || !sensor->initialized) {
        return 0.0f;
    }
    return sensor->angle_rad;
}

float as5600_get_angle_deg(as5600_t* sensor) {
    if (!sensor || !sensor->initialized) {
        return 0.0f;
    }
    return sensor->angle_rad * 180.0f / M_PI;
}

uint16_t as5600_get_raw_angle(as5600_t* sensor) {
    if (!sensor || !sensor->initialized) {
        return 0;
    }
    return sensor->raw_angle;
}

esp_err_t as5600_test_connection(as5600_t* sensor) {
    if (!sensor) {
        return ESP_ERR_INVALID_ARG;
    }

    // Try to read status register
    uint8_t status;
    esp_err_t ret = i2c_bus_read_bytes(sensor->device_handle, AS5600_REG_STATUS, 1, &status);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AS5600 %s connection test failed: %s", sensor->name, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGD(TAG, "AS5600 %s status: 0x%02X", sensor->name, status);
    return ESP_OK;
}

esp_err_t as5600_deinit(as5600_t* sensor) {
    if (!sensor) {
        return ESP_OK;
    }

    if (sensor->device_handle) {
        i2c_bus_device_delete(&sensor->device_handle);
    }

    free(sensor);
    ESP_LOGI(TAG, "AS5600 %s deinitialized", sensor->name);
    
    return ESP_OK;
}
