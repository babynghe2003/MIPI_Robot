/* MPU9250 Sensor Control Implementation - C wrapper for C++ class */

#include "mpu9250_control.h"
#include "tca9548a.h"
#include "esp_log.h"
#include "i2c_bus.h"

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration - we'll use opaque pointer to C++ class
typedef struct mpu9250_handle mpu9250_handle_t;

// External C++ functions - defined in separate C++ file
extern mpu9250_handle_t* mpu9250_cpp_create(i2c_bus_handle_t bus_handle, i2c_bus_device_handle_t device_handle);
extern int mpu9250_cpp_begin(mpu9250_handle_t* handle);
extern int mpu9250_cpp_read_sensor(mpu9250_handle_t* handle);
extern void mpu9250_cpp_get_angles(mpu9250_handle_t* handle, float* x, float* y, float* z);
extern void mpu9250_cpp_get_accel(mpu9250_handle_t* handle, float* x, float* y, float* z);
extern void mpu9250_cpp_get_gyro(mpu9250_handle_t* handle, float* x, float* y, float* z);
extern void mpu9250_cpp_get_mag(mpu9250_handle_t* handle, float* x, float* y, float* z);
extern float mpu9250_cpp_get_temp(mpu9250_handle_t* handle);
extern int mpu9250_cpp_calibrate_gyro(mpu9250_handle_t* handle);
extern int mpu9250_cpp_calibrate_mag(mpu9250_handle_t* handle);
extern void mpu9250_cpp_reset_angles(mpu9250_handle_t* handle);
extern void mpu9250_cpp_destroy(mpu9250_handle_t* handle);

static const char *TAG = "mpu9250_control";
static i2c_bus_handle_t i2c_bus_handle = NULL;
static i2c_bus_device_handle_t mpu_device_handle = NULL;
static mpu9250_handle_t *mpu_handle = NULL;
static tca9548a_handle_t *mux_handle = NULL;
static bool mpu_initialized = false;
static bool use_multiplexer = false;
static uint8_t mux_channel = 0;

esp_err_t mpu9250_init(bool use_mux, uint8_t channel) {
    if (mpu_initialized) {
        ESP_LOGW(TAG, "MPU9250 already initialized");
        return ESP_OK;
    }

    // Store multiplexer settings
    use_multiplexer = use_mux;
    mux_channel = channel;

    // Initialize I2C bus using i2c_bus
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ,
        },
    };

    // Create I2C bus
    i2c_bus_handle = i2c_bus_create(I2C_MASTER_NUM, &conf);
    if (!i2c_bus_handle) {
        ESP_LOGE(TAG, "Failed to create I2C bus");
        return ESP_ERR_INVALID_STATE;
    }

    // Debug: Scan I2C bus to see available devices
    ESP_LOGI(TAG, "Scanning I2C bus before initializing TCA9548A...");
    i2c_scan_bus(i2c_bus_handle);

    // Initialize TCA9548A multiplexer if requested
    if (use_multiplexer) {
        mux_handle = tca9548a_init(i2c_bus_handle, TCA9548A_I2C_ADDR);
        if (!mux_handle) {
            ESP_LOGE(TAG, "Failed to initialize TCA9548A multiplexer");
            i2c_bus_delete(&i2c_bus_handle);
            return ESP_ERR_INVALID_STATE;
        }

        // Select the MPU9250 channel
        esp_err_t ret = tca9548a_select_channel(mux_handle, mux_channel);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to select MPU9250 channel %d", mux_channel);
            tca9548a_deinit(mux_handle);
            mux_handle = NULL;
            i2c_bus_delete(&i2c_bus_handle);
            return ret;
        }

        ESP_LOGI(TAG, "TCA9548A multiplexer initialized, MPU9250 on channel %d", mux_channel);
    }

    // Create MPU9250 device on the bus (address 0x68, default clock speed)
    mpu_device_handle = i2c_bus_device_create(i2c_bus_handle, 0x68, 0);
    if (!mpu_device_handle) {
        ESP_LOGE(TAG, "Failed to create MPU9250 device");
        if (mux_handle) {
            tca9548a_deinit(mux_handle);
            mux_handle = NULL;
        }
        i2c_bus_delete(&i2c_bus_handle);
        return ESP_ERR_INVALID_STATE;
    }

    // Initialize MPU9250 C++ wrapper
    mpu_handle = mpu9250_cpp_create(i2c_bus_handle, mpu_device_handle);
    if (!mpu_handle) {
        ESP_LOGE(TAG, "Failed to create MPU9250 instance");
        i2c_bus_device_delete(&mpu_device_handle);
        if (mux_handle) {
            tca9548a_deinit(mux_handle);
            mux_handle = NULL;
        }
        i2c_bus_delete(&i2c_bus_handle);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = mpu9250_cpp_begin(mpu_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU9250: %s", esp_err_to_name(ret));
        mpu9250_cpp_destroy(mpu_handle);
        mpu_handle = NULL;
        i2c_bus_device_delete(&mpu_device_handle);
        if (mux_handle) {
            tca9548a_deinit(mux_handle);
            mux_handle = NULL;
        }
        i2c_bus_delete(&i2c_bus_handle);
        return ret;
    }

    mpu_initialized = true;
    
    if (use_multiplexer) {
        ESP_LOGI(TAG, "MPU9250 initialized successfully with TCA9548A multiplexer (channel %d) on pins SDA=%d, SCL=%d", 
                 mux_channel, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    } else {
        ESP_LOGI(TAG, "MPU9250 initialized successfully using direct I2C on pins SDA=%d, SCL=%d", 
                 I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    }
    
    return ESP_OK;
}

esp_err_t mpu9250_init_with_bus(i2c_bus_handle_t bus_handle) {
    if (mpu_initialized) {
        ESP_LOGW(TAG, "MPU9250 already initialized");
        return ESP_OK;
    }

    if (!bus_handle) {
        ESP_LOGE(TAG, "Invalid I2C bus handle provided");
        return ESP_ERR_INVALID_ARG;
    }

    // Use the provided I2C bus handle
    i2c_bus_handle = bus_handle;
    use_multiplexer = false;
    mux_channel = 0;

    ESP_LOGI(TAG, "Using shared I2C bus for MPU9250");

    // Create MPU9250 device handle (address 0x68, default clock speed)
    mpu_device_handle = i2c_bus_device_create(i2c_bus_handle, 0x68, 0);
    if (!mpu_device_handle) {
        ESP_LOGE(TAG, "Failed to create MPU9250 device handle");
        return ESP_ERR_INVALID_STATE;
    }

    // Initialize MPU9250 C++ wrapper
    mpu_handle = mpu9250_cpp_create(i2c_bus_handle, mpu_device_handle);
    if (!mpu_handle) {
        ESP_LOGE(TAG, "Failed to create MPU9250 instance");
        i2c_bus_device_delete(&mpu_device_handle);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret = mpu9250_cpp_begin(mpu_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU9250: %s", esp_err_to_name(ret));
        mpu9250_cpp_destroy(mpu_handle);
        mpu_handle = NULL;
        i2c_bus_device_delete(&mpu_device_handle);
        return ret;
    }

    mpu_initialized = true;
    ESP_LOGI(TAG, "MPU9250 initialized successfully with shared bus");
    
    return ESP_OK;
}

esp_err_t mpu9250_init_legacy(void) {
    return mpu9250_init(false, 0);
}

esp_err_t mpu9250_read_sensor(void) {
    if (!mpu_initialized || !mpu_handle) {
        ESP_LOGE(TAG, "MPU9250 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Select MPU9250 channel if using multiplexer
    if (use_multiplexer && mux_handle) {
        esp_err_t ret = tca9548a_select_channel(mux_handle, mux_channel);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to select MPU9250 channel before reading");
            return ret;
        }
    }

    return mpu9250_cpp_read_sensor(mpu_handle);
}

esp_err_t mpu9250_get_angles(mpu9250_angles_t *angles) {
    if (!mpu_initialized || !mpu_handle || !angles) {
        return ESP_ERR_INVALID_ARG;
    }

    mpu9250_cpp_get_angles(mpu_handle, &angles->x, &angles->y, &angles->z);
    return ESP_OK;
}

esp_err_t mpu9250_get_accel(mpu9250_accel_t *accel) {
    if (!mpu_initialized || !mpu_handle || !accel) {
        return ESP_ERR_INVALID_ARG;
    }

    mpu9250_cpp_get_accel(mpu_handle, &accel->x, &accel->y, &accel->z);
    return ESP_OK;
}

esp_err_t mpu9250_get_gyro(mpu9250_gyro_t *gyro) {
    if (!mpu_initialized || !mpu_handle || !gyro) {
        return ESP_ERR_INVALID_ARG;
    }

    mpu9250_cpp_get_gyro(mpu_handle, &gyro->x, &gyro->y, &gyro->z);
    return ESP_OK;
}

esp_err_t mpu9250_get_mag(mpu9250_mag_t *mag) {
    if (!mpu_initialized || !mpu_handle || !mag) {
        return ESP_ERR_INVALID_ARG;
    }

    mpu9250_cpp_get_mag(mpu_handle, &mag->x, &mag->y, &mag->z);
    return ESP_OK;
}

esp_err_t mpu9250_get_temperature(float *temp) {
    if (!mpu_initialized || !mpu_handle || !temp) {
        return ESP_ERR_INVALID_ARG;
    }

    *temp = mpu9250_cpp_get_temp(mpu_handle);
    return ESP_OK;
}

esp_err_t mpu9250_calibrate_gyro(void) {
    if (!mpu_initialized || !mpu_handle) {
        ESP_LOGE(TAG, "MPU9250 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting MPU9250 gyroscope calibration - keep sensor still...");
    esp_err_t ret = mpu9250_cpp_calibrate_gyro(mpu_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "MPU9250 gyroscope calibration completed");
    }
    
    return ret;
}

esp_err_t mpu9250_calibrate_mag(void) {
    if (!mpu_initialized || !mpu_handle) {
        ESP_LOGE(TAG, "MPU9250 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Starting MPU9250 magnetometer calibration - rotate sensor in all directions...");
    esp_err_t ret = mpu9250_cpp_calibrate_mag(mpu_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "MPU9250 magnetometer calibration completed");
    }
    
    return ret;
}

esp_err_t mpu9250_reset_angles(void) {
    if (!mpu_initialized || !mpu_handle) {
        ESP_LOGE(TAG, "MPU9250 not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Resetting MPU9250 angles to current position");
    mpu9250_cpp_reset_angles(mpu_handle);
    ESP_LOGI(TAG, "MPU9250 angles reset completed");
    
    return ESP_OK;
}

esp_err_t mpu9250_deinit(void) {
    if (!mpu_initialized) {
        return ESP_OK;
    }

    if (mpu_handle) {
        mpu9250_cpp_destroy(mpu_handle);
        mpu_handle = NULL;
    }

    if (mpu_device_handle) {
        i2c_bus_device_delete(&mpu_device_handle);
    }

    // Cleanup multiplexer if used
    if (mux_handle) {
        tca9548a_deinit(mux_handle);
        mux_handle = NULL;
    }

    esp_err_t ret = ESP_OK;
    if (i2c_bus_handle) {
        ret = i2c_bus_delete(&i2c_bus_handle);
    }
    
    mpu_initialized = false;
    use_multiplexer = false;
    
    ESP_LOGI(TAG, "MPU9250 deinitialized");
    return ret;
}

#ifdef __cplusplus
}
#endif
