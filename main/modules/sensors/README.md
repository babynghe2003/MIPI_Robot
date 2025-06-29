# Sensors Module

This module will contain sensor drivers and related functionality.

## Planned Sensors:
- MPU6050/MPU9250 (6/9-axis IMU)
- Distance sensors (ultrasonic, lidar)
- Environmental sensors (temperature, humidity)
- Pressure sensors

## Files to add:
- `mpu6050_driver.c/h` - MPU6050 implementation
- `sensor_fusion.c/h` - Sensor data fusion algorithms
- `sensor_task.c` - Sensor reading task
- `sensor_calibration.c/h` - Sensor calibration routines

## Configuration:
Update `app_config.h` to enable:
```c
#define MODULE_SENSORS_ENABLED      1
```
