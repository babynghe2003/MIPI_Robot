/* MPU9250 Sensor Control Implementation - C wrapper for C++ class */

#include "mpu9250_control.h"
#include "esp_log.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

// Forward declaration - we'll use opaque pointer to C++ class
typedef struct mpu9250_handle mpu9250_handle_t;

// External C++ functions - defined in separate C++ file
extern mpu9250_handle_t* mpu9250_cpp_create(int i2c_port);
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
static mpu9250_handle_t *mpu_handle = NULL;
static bool mpu_initialized = false;

esp_err_t mpu9250_init(void) {
    if (mpu_initialized) {
        ESP_LOGW(TAG, "MPU9250 already initialized");
        return ESP_OK;
    }

    // Initialize I2C master
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

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C parameters: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 
                            I2C_MASTER_RX_BUF_DISABLE, 
                            I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize MPU9250
    mpu_handle = mpu9250_cpp_create(I2C_MASTER_NUM);
    if (!mpu_handle) {
        ESP_LOGE(TAG, "Failed to create MPU9250 instance");
        return ESP_ERR_NO_MEM;
    }

    ret = mpu9250_cpp_begin(mpu_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU9250: %s", esp_err_to_name(ret));
        mpu9250_cpp_destroy(mpu_handle);
        mpu_handle = NULL;
        return ret;
    }

    mpu_initialized = true;
    ESP_LOGI(TAG, "MPU9250 initialized successfully on I2C pins SDA=%d, SCL=%d", 
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    return ESP_OK;
}

esp_err_t mpu9250_read_sensor(void) {
    if (!mpu_initialized || !mpu_handle) {
        ESP_LOGE(TAG, "MPU9250 not initialized");
        return ESP_ERR_INVALID_STATE;
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

    esp_err_t ret = i2c_driver_delete(I2C_MASTER_NUM);
    mpu_initialized = false;
    
    ESP_LOGI(TAG, "MPU9250 deinitialized");
    return ret;
}

#ifdef __cplusplus
}
#endif
