/* MPU9250 C++ Bridge - Interface between C and C++ code */

#include "mpu9250_idf.h"
#include "esp_err.h"
#include <cstdlib>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle structure
typedef struct mpu9250_handle {
    MPU9250* mpu_instance;
} mpu9250_handle_t;

// Create MPU9250 instance
mpu9250_handle_t* mpu9250_cpp_create(int i2c_port) {
    mpu9250_handle_t* handle = (mpu9250_handle_t*)malloc(sizeof(mpu9250_handle_t));
    if (!handle) {
        return NULL;
    }
    
    handle->mpu_instance = new MPU9250((i2c_port_t)i2c_port, 0x68);
    if (!handle->mpu_instance) {
        free(handle);
        return NULL;
    }
    
    return handle;
}

// Initialize MPU9250
int mpu9250_cpp_begin(mpu9250_handle_t* handle) {
    if (!handle || !handle->mpu_instance) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return handle->mpu_instance->begin();
}

// Read sensor data
int mpu9250_cpp_read_sensor(mpu9250_handle_t* handle) {
    if (!handle || !handle->mpu_instance) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return handle->mpu_instance->readSensor();
}

// Get filtered angles (degrees)
void mpu9250_cpp_get_angles(mpu9250_handle_t* handle, float* x, float* y, float* z) {
    if (!handle || !handle->mpu_instance || !x || !y || !z) {
        return;
    }
    
    *x = handle->mpu_instance->getAngleX();
    *y = handle->mpu_instance->getAngleY();
    *z = handle->mpu_instance->getAngleZ();
}

// Get accelerometer data (m/s²)
void mpu9250_cpp_get_accel(mpu9250_handle_t* handle, float* x, float* y, float* z) {
    if (!handle || !handle->mpu_instance || !x || !y || !z) {
        return;
    }
    
    *x = handle->mpu_instance->getAccelX_mss();
    *y = handle->mpu_instance->getAccelY_mss();
    *z = handle->mpu_instance->getAccelZ_mss();
}

// Get gyroscope data (rad/s)
void mpu9250_cpp_get_gyro(mpu9250_handle_t* handle, float* x, float* y, float* z) {
    if (!handle || !handle->mpu_instance || !x || !y || !z) {
        return;
    }
    
    *x = handle->mpu_instance->getGyroX_rads();
    *y = handle->mpu_instance->getGyroY_rads();
    *z = handle->mpu_instance->getGyroZ_rads();
}

// Get magnetometer data (µT)
void mpu9250_cpp_get_mag(mpu9250_handle_t* handle, float* x, float* y, float* z) {
    if (!handle || !handle->mpu_instance || !x || !y || !z) {
        return;
    }
    
    *x = handle->mpu_instance->getMagX_uT();
    *y = handle->mpu_instance->getMagY_uT();
    *z = handle->mpu_instance->getMagZ_uT();
}

// Get temperature (°C)
float mpu9250_cpp_get_temp(mpu9250_handle_t* handle) {
    if (!handle || !handle->mpu_instance) {
        return 0.0f;
    }
    
    return handle->mpu_instance->getTemperature_C();
}

// Calibrate gyroscope
int mpu9250_cpp_calibrate_gyro(mpu9250_handle_t* handle) {
    if (!handle || !handle->mpu_instance) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return handle->mpu_instance->calibrateGyro();
}

// Calibrate magnetometer
int mpu9250_cpp_calibrate_mag(mpu9250_handle_t* handle) {
    if (!handle || !handle->mpu_instance) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return handle->mpu_instance->calibrateMag();
}

// Reset angles to current accelerometer reading
void mpu9250_cpp_reset_angles(mpu9250_handle_t* handle) {
    if (!handle || !handle->mpu_instance) {
        return;
    }
    
    handle->mpu_instance->resetAngles();
}

// Destroy MPU9250 instance
void mpu9250_cpp_destroy(mpu9250_handle_t* handle) {
    if (!handle) {
        return;
    }
    
    if (handle->mpu_instance) {
        delete handle->mpu_instance;
        handle->mpu_instance = NULL;
    }
    
    free(handle);
}

#ifdef __cplusplus
}
#endif
