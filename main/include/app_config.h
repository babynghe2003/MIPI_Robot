/* Application Configuration Header */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "sdkconfig.h"

/* ========== Hardware Configuration ========== */
// GPIO Definitions
#define SERVO_GPIO_PIN              11    // Servo motor pin
#define LED_GPIO_PIN                CONFIG_BLINK_GPIO  // LED pin

// I2C Configuration (for MPU9250 and other sensors with TCA9548A multiplexer)
#define I2C_MASTER_SCL_IO           GPIO_NUM_9     // I2C master clock (ESP32-S3)
#define I2C_MASTER_SDA_IO           GPIO_NUM_8     // I2C master data (ESP32-S3)
#define I2C_MASTER_NUM              I2C_NUM_0     // I2C master i2c port number
#define I2C_MASTER_FREQ_HZ          100000  // I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE   0     // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0     // I2C master doesn't need buffer
#define I2C_MASTER_TIMEOUT_MS       1000

// TCA9548A I2C Multiplexer Configuration
#define TCA9548A_I2C_ADDR           0x70   // TCA9548A I2C address
#define MPU9250_MUX_CHANNEL         1      // MPU9250 is on channel 1

// I2S Configuration (for future audio)
#define I2S_SAMPLE_RATE             16000  // Sample rate for audio
#define I2S_BITS_PER_SAMPLE         16     // Bits per sample

/* ========== Task Configuration ========== */
// Task Stack Sizes
#define TASK_STACK_SIZE_SMALL       2048
#define TASK_STACK_SIZE_MEDIUM      4096
#define TASK_STACK_SIZE_LARGE       8192

// Task Priorities
#define TASK_PRIORITY_LOW           5
#define TASK_PRIORITY_NORMAL        10
#define TASK_PRIORITY_HIGH          15

// Task Delays
#define TASK_DELAY_MS_FAST          50
#define TASK_DELAY_MS_NORMAL        100
#define TASK_DELAY_MS_SLOW          1000

/* ========== Module Enable/Disable ========== */
#define MODULE_SERVO_ENABLED        0
#define MODULE_LED_ENABLED          1
#define MODULE_SENSORS_ENABLED      1  // Enable when MPU is added
#define MODULE_BLDC_ENABLED         1  // Enable BLDC motor control
#define MODULE_AUDIO_ENABLED        0  // Enable when I2S mic is added
#define MODULE_WIFI_ENABLED         0  // Enable when WiFi is needed

/* ========== Debug Configuration ========== */
#define DEBUG_TASK_HEARTBEAT        1  // Enable task heartbeat logs
#define DEBUG_SENSOR_DATA           1  // Enable sensor data logs

#endif // APP_CONFIG_H
