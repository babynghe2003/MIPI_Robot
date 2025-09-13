#include "mpu9250.h"
#include "esp_log.h"
#include "i2c_bus.h"
#include <math.h>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

static const char *TAG = "MPU9250";

// MPU9250 register addresses
#define MPU9250_SELF_TEST_X_GYRO     0x00
#define MPU9250_SELF_TEST_Y_GYRO     0x01
#define MPU9250_SELF_TEST_Z_GYRO     0x02
#define MPU9250_SELF_TEST_X_ACCEL    0x0D
#define MPU9250_SELF_TEST_Y_ACCEL    0x0E
#define MPU9250_SELF_TEST_Z_ACCEL    0x0F
#define MPU9250_XG_OFFSET_H          0x13
#define MPU9250_XG_OFFSET_L          0x14
#define MPU9250_YG_OFFSET_H          0x15
#define MPU9250_YG_OFFSET_L          0x16
#define MPU9250_ZG_OFFSET_H          0x17
#define MPU9250_ZG_OFFSET_L          0x18
#define MPU9250_SMPLRT_DIV           0x19
#define MPU9250_CONFIG               0x1A
#define MPU9250_GYRO_CONFIG          0x1B
#define MPU9250_ACCEL_CONFIG         0x1C
#define MPU9250_ACCEL_CONFIG2        0x1D
#define MPU9250_LP_ACCEL_ODR         0x1E
#define MPU9250_WOM_THR              0x1F
#define MPU9250_FIFO_EN              0x23
#define MPU9250_I2C_MST_CTRL         0x24
#define MPU9250_I2C_SLV0_ADDR        0x25
#define MPU9250_I2C_SLV0_REG         0x26
#define MPU9250_I2C_SLV0_CTRL        0x27
#define MPU9250_I2C_SLV1_ADDR        0x28
#define MPU9250_I2C_SLV1_REG         0x29
#define MPU9250_I2C_SLV1_CTRL        0x2A
#define MPU9250_I2C_SLV2_ADDR        0x2B
#define MPU9250_I2C_SLV2_REG         0x2C
#define MPU9250_I2C_SLV2_CTRL        0x2D
#define MPU9250_I2C_SLV3_ADDR        0x2E
#define MPU9250_I2C_SLV3_REG         0x2F
#define MPU9250_I2C_SLV3_CTRL        0x30
#define MPU9250_I2C_SLV4_ADDR        0x31
#define MPU9250_I2C_SLV4_REG         0x32
#define MPU9250_I2C_SLV4_DO          0x33
#define MPU9250_I2C_SLV4_CTRL        0x34
#define MPU9250_I2C_SLV4_DI          0x35
#define MPU9250_I2C_MST_STATUS       0x36
#define MPU9250_INT_PIN_CFG          0x37
#define MPU9250_INT_ENABLE           0x38
#define MPU9250_INT_STATUS           0x3A
#define MPU9250_ACCEL_XOUT_H         0x3B
#define MPU9250_ACCEL_XOUT_L         0x3C
#define MPU9250_ACCEL_YOUT_H         0x3D
#define MPU9250_ACCEL_YOUT_L         0x3E
#define MPU9250_ACCEL_ZOUT_H         0x3F
#define MPU9250_ACCEL_ZOUT_L         0x40
#define MPU9250_TEMP_OUT_H           0x41
#define MPU9250_TEMP_OUT_L           0x42
#define MPU9250_GYRO_XOUT_H          0x43
#define MPU9250_GYRO_XOUT_L          0x44
#define MPU9250_GYRO_YOUT_H          0x45
#define MPU9250_GYRO_YOUT_L          0x46
#define MPU9250_GYRO_ZOUT_H          0x47
#define MPU9250_GYRO_ZOUT_L          0x48
#define MPU9250_EXT_SENS_DATA_00     0x49
#define MPU9250_EXT_SENS_DATA_01     0x4A
#define MPU9250_EXT_SENS_DATA_02     0x4B
#define MPU9250_EXT_SENS_DATA_03     0x4C
#define MPU9250_EXT_SENS_DATA_04     0x4D
#define MPU9250_EXT_SENS_DATA_05     0x4E
#define MPU9250_EXT_SENS_DATA_06     0x4F
#define MPU9250_EXT_SENS_DATA_07     0x50
#define MPU9250_EXT_SENS_DATA_08     0x51
#define MPU9250_EXT_SENS_DATA_09     0x52
#define MPU9250_EXT_SENS_DATA_10     0x53
#define MPU9250_EXT_SENS_DATA_11     0x54
#define MPU9250_EXT_SENS_DATA_12     0x55
#define MPU9250_EXT_SENS_DATA_13     0x56
#define MPU9250_EXT_SENS_DATA_14     0x57
#define MPU9250_EXT_SENS_DATA_15     0x58
#define MPU9250_EXT_SENS_DATA_16     0x59
#define MPU9250_EXT_SENS_DATA_17     0x5A
#define MPU9250_EXT_SENS_DATA_18     0x5B
#define MPU9250_EXT_SENS_DATA_19     0x5C
#define MPU9250_EXT_SENS_DATA_20     0x5D
#define MPU9250_EXT_SENS_DATA_21     0x5E
#define MPU9250_EXT_SENS_DATA_22     0x5F
#define MPU9250_EXT_SENS_DATA_23     0x60
#define MPU9250_I2C_SLV0_DO          0x63
#define MPU9250_I2C_SLV1_DO          0x64
#define MPU9250_I2C_SLV2_DO          0x65
#define MPU9250_I2C_SLV3_DO          0x66
#define MPU9250_I2C_MST_DELAY_CTRL   0x67
#define MPU9250_SIGNAL_PATH_RESET    0x68
#define MPU9250_MOT_DETECT_CTRL      0x69
#define MPU9250_USER_CTRL            0x6A
#define MPU9250_PWR_MGMT_1           0x6B
#define MPU9250_PWR_MGMT_2           0x6C
#define MPU9250_FIFO_COUNTH          0x72
#define MPU9250_FIFO_COUNTL          0x73
#define MPU9250_FIFO_R_W             0x74
#define MPU9250_WHO_AM_I             0x75

// AK8963 (Magnetometer) registers
#define AK8963_ADDR                  0x0C
#define AK8963_WHO_AM_I              0x00
#define AK8963_INFO                  0x01
#define AK8963_ST1                   0x02
#define AK8963_XOUT_L                0x03
#define AK8963_XOUT_H                0x04
#define AK8963_YOUT_L                0x05
#define AK8963_YOUT_H                0x06
#define AK8963_ZOUT_L                0x07
#define AK8963_ZOUT_H                0x08
#define AK8963_ST2                   0x09
#define AK8963_CNTL1                 0x0A
#define AK8963_CNTL2                 0x0B
#define AK8963_ASTC                  0x0C
#define AK8963_I2CDIS                0x0F
#define AK8963_ASAX                  0x10
#define AK8963_ASAY                  0x11
#define AK8963_ASAZ                  0x12

MPU9250::MPU9250(i2c_bus_handle_t bus_handle)
{
    _bus_handle=bus_handle;
    _device_handle = i2c_bus_device_create(_bus_handle, 0x68, 0);
    _mag_device_handle = NULL;
    // Initialize variables
    _accCoef = 0.02f;
    _gyroCoef = 0.98f;
    
    // Set default scale factors
    _accelScale = 2.0f / 32768.0f * 9.80665f; // ±2g range in m/s²
    _gyroScale = 250.0f / 32768.0f * M_PI / 180.0f; // ±250 dps range in rad/s
    _magScale = 4912.0f / 32760.0f; // ±4912 µT range
    
    // Initialize calibration values
    _gyroBiasX = _gyroBiasY = _gyroBiasZ = 0.0f;
    _magBiasX = _magBiasY = _magBiasZ = 0.0f;
    _magScaleFactorX = _magScaleFactorY = _magScaleFactorZ = 1.0f;
    
    // Initialize angles
    _angleX = _angleY = _angleZ = 0.0f;
    _angleAccX = _angleAccY = 0.0f;
    _angleGyroX = _angleGyroY = _angleGyroZ = 0.0f;
    
    _preInterval = 0;
    _interval = 0.0f;
}

esp_err_t MPU9250::begin() {
    ESP_LOGI(TAG, "Initializing MPU9250...");
    
    // Reset device
    esp_err_t ret = writeByte(MPU9250_PWR_MGMT_1, 0x80);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset MPU9250");
        return ret;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Wake up device and select best clock source
    ret = writeByte(MPU9250_PWR_MGMT_1, 0x01);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU9250");
        return ret;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Check WHO_AM_I register
    uint8_t whoAmI = readByte(MPU9250_WHO_AM_I);
    if (whoAmI != 0x71 && whoAmI != 0x70 && whoAmI != 0x68) {
        ESP_LOGE(TAG, "MPU9250/MPU6050 WHO_AM_I check failed: 0x%02X", whoAmI);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "IMU WHO_AM_I: 0x%02X %s", whoAmI, 
             (whoAmI == 0x71) ? "(MPU9250)" : 
             (whoAmI == 0x70) ? "(MPU9250/MPU6500)" : "(MPU6050)");
    
    // Store device type for later use
    _deviceType = whoAmI;
    
    // Configure accelerometer, gyroscope, and magnetometer
    ret = configureAccel();
    if (ret != ESP_OK) return ret;
    
    ret = configureGyro();
    if (ret != ESP_OK) return ret;
    
    // Only configure magnetometer for MPU9250 (not MPU6050/6500)
    if (_deviceType == 0x71) {
        ret = configureMag();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Magnetometer initialization failed, continuing without mag data");
        }
    } else {
        ESP_LOGI(TAG, "MPU6050/6500 detected - magnetometer not available");
        // Set magnetometer values to zero for MPU6050
        _magX = _magY = _magZ = 0.0f;
    }
    
    // Initialize timing
    _preInterval = esp_timer_get_time() / 1000;
    
    ESP_LOGI(TAG, "MPU9250 initialization complete");
    return ESP_OK;
}

esp_err_t MPU9250::configureAccel() {
    // Set accelerometer full scale range to ±2g
    esp_err_t ret = writeByte(MPU9250_ACCEL_CONFIG, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accelerometer");
        return ret;
    }
    
    // Set accelerometer bandwidth to 41 Hz
    ret = writeByte(MPU9250_ACCEL_CONFIG2, 0x03);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accelerometer bandwidth");
        return ret;
    }
    
    return ESP_OK;
}

esp_err_t MPU9250::configureGyro() {
    // Set gyroscope full scale range to ±250 dps
    esp_err_t ret = writeByte(MPU9250_GYRO_CONFIG, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyroscope");
        return ret;
    }
    
    // Set bandwidth to 41 Hz
    ret = writeByte(MPU9250_CONFIG, 0x03);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyroscope bandwidth");
        return ret;
    }
    
    return ESP_OK;
}

esp_err_t MPU9250::configureMag() {
    // Create magnetometer device handle
    _mag_device_handle = i2c_bus_device_create(_bus_handle, AK8963_ADDR, 0);
    if (!_mag_device_handle) {
        ESP_LOGE(TAG, "Failed to create AK8963 device handle");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Enable I2C master mode
    esp_err_t ret = writeByte(MPU9250_USER_CTRL, 0x20);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2C master mode");
        return ret;
    }
    
    // Set I2C master clock to 400 kHz
    ret = writeByte(MPU9250_I2C_MST_CTRL, 0x0D);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set I2C master clock");
        return ret;
    }
    
    // Enable bypass mode to access magnetometer
    ret = writeByte(MPU9250_INT_PIN_CFG, 0x02);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable bypass mode");
        return ret;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // Check magnetometer WHO_AM_I
    uint8_t magWhoAmI = readMagByte(AK8963_WHO_AM_I);
    if (magWhoAmI != 0x48) {
        ESP_LOGE(TAG, "AK8963 WHO_AM_I check failed: 0x%02X", magWhoAmI);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "AK8963 WHO_AM_I: 0x%02X", magWhoAmI);
    
    // Power down magnetometer
    ret = writeMagByte(AK8963_CNTL1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to power down magnetometer");
        return ret;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // Set magnetometer to fuse ROM access mode
    ret = writeMagByte(AK8963_CNTL1, 0x0F);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set magnetometer to fuse ROM mode");
        return ret;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // Read fuse ROM data for sensitivity adjustment
    uint8_t fuseROM[3];
    ret = readMagBytes(AK8963_ASAX, fuseROM, 3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read magnetometer fuse ROM");
        return ret;
    }
    
    // Calculate magnetometer sensitivity adjustment values
    float magAdjX = (((float)fuseROM[0]) - 128.0f) / 256.0f + 1.0f;
    float magAdjY = (((float)fuseROM[1]) - 128.0f) / 256.0f + 1.0f;
    float magAdjZ = (((float)fuseROM[2]) - 128.0f) / 256.0f + 1.0f;
    
    ESP_LOGI(TAG, "Magnetometer sensitivity adjustments: X=%.3f, Y=%.3f, Z=%.3f", 
             magAdjX, magAdjY, magAdjZ);
    
    // Power down magnetometer
    ret = writeMagByte(AK8963_CNTL1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to power down magnetometer");
        return ret;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // Set magnetometer to continuous measurement mode (16-bit, 100Hz)
    ret = writeMagByte(AK8963_CNTL1, 0x16);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set magnetometer to continuous mode");
        return ret;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // Apply sensitivity adjustments to scale factor
    _magScale = 4912.0f / 32760.0f;
    
    return ESP_OK;
}

esp_err_t MPU9250::readSensor() {
    // Read accelerometer and gyroscope data (14 bytes)
    uint8_t buffer[14];
    esp_err_t ret = readBytes(MPU9250_ACCEL_XOUT_H, buffer, 14);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read accelerometer/gyroscope data");
        return ret;
    }
    
    // Parse accelerometer data
    _rawAccelX = (int16_t)((buffer[0] << 8) | buffer[1]);
    _rawAccelY = (int16_t)((buffer[2] << 8) | buffer[3]);
    _rawAccelZ = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    // Parse temperature data
    _rawTemp = (int16_t)((buffer[6] << 8) | buffer[7]);
    
    // Parse gyroscope data
    _rawGyroX = (int16_t)((buffer[8] << 8) | buffer[9]);
    _rawGyroY = (int16_t)((buffer[10] << 8) | buffer[11]);
    _rawGyroZ = (int16_t)((buffer[12] << 8) | buffer[13]);
    
    // Read magnetometer data (only for MPU9250)
    if (_deviceType == 0x71) {
        uint8_t magBuffer[7];
        ret = readMagBytes(AK8963_XOUT_L, magBuffer, 7);
        if (ret == ESP_OK) {
            // Check if magnetometer data is ready (bit 0 of ST2 should be 0)
            if (!(magBuffer[6] & 0x08)) {
                // Parse magnetometer data (little endian)
                _rawMagX = (int16_t)((magBuffer[1] << 8) | magBuffer[0]);
                _rawMagY = (int16_t)((magBuffer[3] << 8) | magBuffer[2]);
                _rawMagZ = (int16_t)((magBuffer[5] << 8) | magBuffer[4]);
            }
        } else {
            // Set magnetometer to zero if read fails
            _rawMagX = _rawMagY = _rawMagZ = 0;
        }
    } else {
        // No magnetometer for MPU6050/6500
        _rawMagX = _rawMagY = _rawMagZ = 0;
    }
    
    // Process all sensor data
    processAccelData();
    processGyroData();
    processMagData();
    calculateAngles();
    
    return ESP_OK;
}

void MPU9250::processAccelData() {
    _accelX = (float)_rawAccelX * _accelScale;
    _accelY = (float)_rawAccelY * _accelScale;
    _accelZ = (float)_rawAccelZ * _accelScale;
    
    _temperature = ((float)_rawTemp - 21.0f) / 333.87f + 21.0f;
}

void MPU9250::processGyroData() {
    _gyroX = (float)_rawGyroX * _gyroScale - _gyroBiasX;
    _gyroY = (float)_rawGyroY * _gyroScale - _gyroBiasY;
    _gyroZ = (float)_rawGyroZ * _gyroScale - _gyroBiasZ;
}

void MPU9250::processMagData() {
    _magX = ((float)_rawMagX * _magScale - _magBiasX) * _magScaleFactorX;
    _magY = ((float)_rawMagY * _magScale - _magBiasY) * _magScaleFactorY;
    _magZ = ((float)_rawMagZ * _magScale - _magBiasZ) * _magScaleFactorZ;
}

void MPU9250::calculateAngles() {
    // Calculate time interval
    int64_t now = esp_timer_get_time() / 1000;
    _interval = (now - _preInterval) * 0.001f;
    
    // Limit interval to prevent large jumps
    if (_interval > 0.1f) _interval = 0.1f;
    if (_interval < 0.0001f) _interval = 0.0001f; // Smaller minimum for high-frequency updates
    
    // Calculate accelerometer angles (in degrees)
    float accMagnitude = sqrtf(_accelX*_accelX + _accelY*_accelY + _accelZ*_accelZ);
    bool accValid = (accMagnitude > 7.0f && accMagnitude < 15.0f); // Expanded valid gravity range
    
    if (accValid) {
        _angleAccX = atan2f(_accelY, sqrtf(_accelZ*_accelZ + _accelX*_accelX)) * 180.0f / M_PI;
        _angleAccY = atan2f(-_accelX, sqrtf(_accelZ*_accelZ + _accelY*_accelY)) * 180.0f / M_PI;
    }
    
    // Calculate gyroscope angular velocities in degrees/s
    float gyroXDeg = _gyroX * 180.0f / M_PI;
    float gyroYDeg = _gyroY * 180.0f / M_PI;
    float gyroZDeg = _gyroZ * 180.0f / M_PI;
    
    // Integrate gyroscope data
    _angleGyroX += gyroXDeg * _interval;
    _angleGyroY += gyroYDeg * _interval;
    _angleGyroZ += gyroZDeg * _interval;
    
    // Adjust filter coefficients based on motion
    float adaptiveAccCoef = 0.002f;
    float adaptiveGyroCoef = 0.998f;
    
    
    // Apply adaptive complementary filter
    if (accValid) {
        _angleX = adaptiveGyroCoef * (_angleX + gyroXDeg * _interval) + adaptiveAccCoef * _angleAccX;
        _angleY = adaptiveGyroCoef * (_angleY + gyroYDeg * _interval) + adaptiveAccCoef * _angleAccY;
    } else {
        // If accelerometer is invalid, use pure gyroscope integration
        _angleX = _angleX + gyroXDeg * _interval;
        _angleY = _angleY + gyroYDeg * _interval;
    }
    
    // Z-axis angle (yaw) from gyroscope only
    _angleZ = _angleGyroZ;
    
    // Apply angle wrapping to prevent overflow
    while (_angleX > 180.0f) _angleX -= 360.0f;
    while (_angleX < -180.0f) _angleX += 360.0f;
    while (_angleY > 180.0f) _angleY -= 360.0f;
    while (_angleY < -180.0f) _angleY += 360.0f;
    while (_angleZ > 180.0f) _angleZ -= 360.0f;
    while (_angleZ < -180.0f) _angleZ += 360.0f;
    
    _preInterval = now;
}

esp_err_t MPU9250::calibrateGyro() {
    ESP_LOGI(TAG, "Starting gyroscope calibration...");
    
    float sumX = 0, sumY = 0, sumZ = 0;
    int samples = 1000;
    int validSamples = 0;
    
    for (int i = 0; i < samples; i++) {
        uint8_t buffer[6];
        esp_err_t ret = readBytes(MPU9250_GYRO_XOUT_H, buffer, 6);
        if (ret == ESP_OK) {
            int16_t gx = (int16_t)((buffer[0] << 8) | buffer[1]);
            int16_t gy = (int16_t)((buffer[2] << 8) | buffer[3]);
            int16_t gz = (int16_t)((buffer[4] << 8) | buffer[5]);
            
            float gxf = (float)gx * _gyroScale;
            float gyf = (float)gy * _gyroScale;
            float gzf = (float)gz * _gyroScale;
            
            // Only use stable samples
            if (fabsf(gxf) < 2.1f && fabsf(gyf) < 2.1f && fabsf(gzf) < 2.1f) {
                sumX += gxf;
                sumY += gyf;
                sumZ += gzf;
                validSamples++;
            }
        }
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
    
    if (validSamples > 0) {
        _gyroBiasX = sumX / validSamples;
        _gyroBiasY = sumY / validSamples;
        _gyroBiasZ = sumZ / validSamples;
        
        ESP_LOGI(TAG, "Gyroscope calibration complete: X=%.6f, Y=%.6f, Z=%.6f rad/s", 
                 _gyroBiasX, _gyroBiasY, _gyroBiasZ);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Gyroscope calibration failed - no stable samples");
        return ESP_ERR_INVALID_STATE;
    }
}

esp_err_t MPU9250::calibrateMag() {
    ESP_LOGI(TAG, "Starting magnetometer calibration - rotate sensor in all directions...");
    
    float magXmin = 1000, magXmax = -1000;
    float magYmin = 1000, magYmax = -1000;  
    float magZmin = 1000, magZmax = -1000;
    
    int samples = 2000;
    for (int i = 0; i < samples; i++) {
        readSensor();
        
        if (_magX < magXmin) magXmin = _magX;
        if (_magX > magXmax) magXmax = _magX;
        if (_magY < magYmin) magYmin = _magY;
        if (_magY > magYmax) magYmax = _magY;
        if (_magZ < magZmin) magZmin = _magZ;
        if (_magZ > magZmax) magZmax = _magZ;
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    // Calculate bias and scale factors
    _magBiasX = (magXmax + magXmin) / 2.0f;
    _magBiasY = (magYmax + magYmin) / 2.0f;
    _magBiasZ = (magZmax + magZmin) / 2.0f;
    
    float magXrange = magXmax - magXmin;
    float magYrange = magYmax - magYmin;
    float magZrange = magZmax - magZmin;
    
    float avgRange = (magXrange + magYrange + magZrange) / 3.0f;
    
    _magScaleFactorX = avgRange / magXrange;
    _magScaleFactorY = avgRange / magYrange;
    _magScaleFactorZ = avgRange / magZrange;
    
    ESP_LOGI(TAG, "Magnetometer calibration complete:");
    ESP_LOGI(TAG, "  Bias: X=%.2f, Y=%.2f, Z=%.2f uT", _magBiasX, _magBiasY, _magBiasZ);
    ESP_LOGI(TAG, "  Scale: X=%.3f, Y=%.3f, Z=%.3f", _magScaleFactorX, _magScaleFactorY, _magScaleFactorZ);
    
    return ESP_OK;
}

void MPU9250::resetAngles() {
    readSensor();
    _angleX = _angleAccX;
    _angleY = _angleAccY;
    _angleZ = 0.0f;
    _angleGyroX = _angleGyroY = _angleGyroZ = 0.0f;
    _preInterval = esp_timer_get_time() / 1000;
}

esp_err_t MPU9250::writeByte(uint8_t reg, uint8_t data) {
    return i2c_bus_write_bytes(_device_handle, reg, 1, &data);
}

uint8_t MPU9250::readByte(uint8_t reg) {
    uint8_t data;
    esp_err_t ret = i2c_bus_read_bytes(_device_handle, reg, 1, &data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read byte from reg 0x%02X", reg);
        return 0;
    }
    return data;
}

esp_err_t MPU9250::readBytes(uint8_t reg, uint8_t *buffer, size_t len) {
    return i2c_bus_read_bytes(_device_handle, reg, len, buffer);
}

esp_err_t MPU9250::writeMagByte(uint8_t reg, uint8_t data) {
    if (!_mag_device_handle) {
        ESP_LOGE(TAG, "Magnetometer device not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_bus_write_bytes(_mag_device_handle, reg, 1, &data);
}

uint8_t MPU9250::readMagByte(uint8_t reg) {
    if (!_mag_device_handle) {
        ESP_LOGE(TAG, "Magnetometer device not initialized");
        return 0;
    }
    uint8_t data;
    esp_err_t ret = i2c_bus_read_bytes(_mag_device_handle, reg, 1, &data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read mag byte from reg 0x%02X", reg);
        return 0;
    }
    return data;
}

esp_err_t MPU9250::readMagBytes(uint8_t reg, uint8_t *buffer, size_t len) {
    if (!_mag_device_handle) {
        ESP_LOGE(TAG, "Magnetometer device not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_bus_read_bytes(_mag_device_handle, reg, len, buffer);
}

MPU9250::~MPU9250() {
    // Clean up magnetometer device handle
    if (_mag_device_handle) {
        i2c_bus_device_delete(&_mag_device_handle);
    }
}
