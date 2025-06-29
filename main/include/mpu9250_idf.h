#pragma once

#include <stdint.h>
#include "i2c_bus.h"
#include "esp_err.h"

class MPU9250 {
public:
    // Constructor
    MPU9250(i2c_bus_handle_t bus_handle, i2c_bus_device_handle_t device_handle);
    
    // Destructor
    ~MPU9250();

    // Core functions
    esp_err_t begin();
    esp_err_t readSensor();
    
    // Accelerometer functions (m/s/s)
    float getAccelX_mss() { return _accelX; }
    float getAccelY_mss() { return _accelY; }
    float getAccelZ_mss() { return _accelZ; }
    
    // Gyroscope functions (rad/s)
    float getGyroX_rads() { return _gyroX; }
    float getGyroY_rads() { return _gyroY; }
    float getGyroZ_rads() { return _gyroZ; }
    
    // Magnetometer functions (uT)
    float getMagX_uT() { return _magX; }
    float getMagY_uT() { return _magY; }
    float getMagZ_uT() { return _magZ; }
    
    // Temperature function (C)
    float getTemperature_C() { return _temperature; }
    
    // Calibration functions
    esp_err_t calibrateGyro();
    esp_err_t calibrateMag();
    
    // Bias getters/setters for gyro
    float getGyroBiasX_rads() { return _gyroBiasX; }
    float getGyroBiasY_rads() { return _gyroBiasY; }
    float getGyroBiasZ_rads() { return _gyroBiasZ; }
    
    void setGyroBiasX_rads(float bias) { _gyroBiasX = bias; }
    void setGyroBiasY_rads(float bias) { _gyroBiasY = bias; }
    void setGyroBiasZ_rads(float bias) { _gyroBiasZ = bias; }
    
    // Bias getters/setters for magnetometer
    float getMagBiasX_uT() { return _magBiasX; }
    float getMagBiasY_uT() { return _magBiasY; }
    float getMagBiasZ_uT() { return _magBiasZ; }
    
    void setMagBiasX_uT(float bias) { _magBiasX = bias; }
    void setMagBiasY_uT(float bias) { _magBiasY = bias; }
    void setMagBiasZ_uT(float bias) { _magBiasZ = bias; }
    
    // Scale factor getters/setters for magnetometer
    float getMagScaleFactorX() { return _magScaleFactorX; }
    float getMagScaleFactorY() { return _magScaleFactorY; }
    float getMagScaleFactorZ() { return _magScaleFactorZ; }
    
    void setMagScaleFactorX(float scale) { _magScaleFactorX = scale; }
    void setMagScaleFactorY(float scale) { _magScaleFactorY = scale; }
    void setMagScaleFactorZ(float scale) { _magScaleFactorZ = scale; }
    
    // Angle calculation functions (degrees)
    float getAngleX() { return _angleX; }
    float getAngleY() { return _angleY; }
    float getAngleZ() { return _angleZ; }
    
    void resetAngles();

private:
    // I2C configuration
    i2c_bus_handle_t _bus_handle;
    i2c_bus_device_handle_t _device_handle;
    i2c_bus_device_handle_t _mag_device_handle;  // For AK8963 magnetometer
    uint8_t _deviceType;  // Store detected device type (0x68, 0x70, 0x71)
    
    // Raw sensor data
    int16_t _rawAccelX, _rawAccelY, _rawAccelZ;
    int16_t _rawGyroX, _rawGyroY, _rawGyroZ;
    int16_t _rawMagX, _rawMagY, _rawMagZ;
    int16_t _rawTemp;
    
    // Processed sensor data
    float _accelX, _accelY, _accelZ;
    float _gyroX, _gyroY, _gyroZ;
    float _magX, _magY, _magZ;
    float _temperature;
    
    // Calculated angles (degrees)
    float _angleX, _angleY, _angleZ;
    float _angleAccX, _angleAccY;
    float _angleGyroX, _angleGyroY, _angleGyroZ;
    
    // Calibration data
    float _gyroBiasX, _gyroBiasY, _gyroBiasZ;
    float _magBiasX, _magBiasY, _magBiasZ;
    float _magScaleFactorX, _magScaleFactorY, _magScaleFactorZ;
    
    // Complementary filter coefficients
    float _accCoef, _gyroCoef;
    
    // Timing
    float _interval;
    int64_t _preInterval;
    
    // Scale factors
    float _accelScale;
    float _gyroScale;
    float _magScale;
    
    // I2C communication functions
    esp_err_t writeByte(uint8_t reg, uint8_t data);
    esp_err_t readBytes(uint8_t reg, uint8_t *buffer, size_t len);
    uint8_t readByte(uint8_t reg);
    
    // Magnetometer I2C functions (AK8963)
    esp_err_t writeMagByte(uint8_t reg, uint8_t data);
    esp_err_t readMagBytes(uint8_t reg, uint8_t *buffer, size_t len);
    uint8_t readMagByte(uint8_t reg);
    
    // Configuration functions
    esp_err_t configureAccel();
    esp_err_t configureGyro();
    esp_err_t configureMag();
    
    // Data processing functions
    void processAccelData();
    void processGyroData();
    void processMagData();
    void calculateAngles();
};
