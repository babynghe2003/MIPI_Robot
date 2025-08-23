/* TCA9548A I2C Multiplexer Control Implementation */

#include "tca9548a.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "TCA9548A";

tca9548a_handle_t* tca9548a_init(i2c_bus_handle_t bus_handle, uint8_t addr) {
    if (!bus_handle) {
        ESP_LOGE(TAG, "Invalid I2C bus handle");
        return NULL;
    }

    // Allocate handle structure
    tca9548a_handle_t* handle = (tca9548a_handle_t*)malloc(sizeof(tca9548a_handle_t));
    if (!handle) {
        ESP_LOGE(TAG, "Failed to allocate memory for handle");
        return NULL;
    }

    // Initialize handle
    memset(handle, 0, sizeof(tca9548a_handle_t));
    handle->bus_handle = bus_handle;
    handle->current_channel = TCA9548A_CHANNEL_NONE;

    // Create device handle for TCA9548A
    handle->device_handle = i2c_bus_device_create(bus_handle, addr, 0);
    if (!handle->device_handle) {
        ESP_LOGE(TAG, "Failed to create TCA9548A device handle at address 0x%02X", addr);
        free(handle);
        return NULL;
    }

    // Test communication by writing to disable all channels (send 0x00)
    uint8_t test_byte = 0x00;
    esp_err_t ret = i2c_bus_write_bytes(handle->device_handle, NULL_I2C_MEM_ADDR, 1, &test_byte);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to communicate with TCA9548A at address 0x%02X: %s", addr, esp_err_to_name(ret));
        i2c_bus_device_delete(&handle->device_handle);
        free(handle);
        return NULL;
    }

    handle->initialized = true;
    handle->current_channel = TCA9548A_CHANNEL_NONE;
    ESP_LOGI(TAG, "TCA9548A initialized successfully at address 0x%02X", addr);
    
    return handle;
}

esp_err_t tca9548a_select_channel(tca9548a_handle_t* handle, uint8_t channel) {
    if (!handle || !handle->initialized) {
        ESP_LOGE(TAG, "TCA9548A not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t channel_byte;
    
    if (channel == TCA9548A_CHANNEL_NONE) {
        // Disable all channels
        channel_byte = 0x00;
    } else if (channel <= 7) {
        // Enable specific channel (bit position)
        channel_byte = 1 << channel;
    } else {
        ESP_LOGE(TAG, "Invalid channel: %d (must be 0-7 or TCA9548A_CHANNEL_NONE)", channel);
        return ESP_ERR_INVALID_ARG;
    }

    // Send channel selection command to TCA9548A
    esp_err_t ret = i2c_bus_write_bytes(handle->device_handle, NULL_I2C_MEM_ADDR, 1, &channel_byte);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to select channel %d: %s", channel, esp_err_to_name(ret));
        return ret;
    }

    handle->current_channel = channel;
    
    if (channel == TCA9548A_CHANNEL_NONE) {
        ESP_LOGD(TAG, "All channels disabled");
    } else {
        ESP_LOGD(TAG, "Channel %d selected", channel);
    }
    
    return ESP_OK;
}

esp_err_t tca9548a_deinit(tca9548a_handle_t* handle) {
    if (!handle) {
        return ESP_OK;
    }

    // Disable all channels before cleanup
    if (handle->initialized && handle->device_handle) {
        tca9548a_select_channel(handle, TCA9548A_CHANNEL_NONE);
        i2c_bus_device_delete(&handle->device_handle);
    }

    free(handle);
    ESP_LOGI(TAG, "TCA9548A deinitialized");
    
    return ESP_OK;
}
