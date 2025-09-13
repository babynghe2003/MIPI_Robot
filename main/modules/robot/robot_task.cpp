// C++ includes
#include <cstdio>
#include <cstring>
#include <string>

// ESP-IDF and project includes
#include "app_config.h"
#include "cas5600.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_simplefoc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mpu9250.h"
#include "pid.h"
#include "sensors/Encoder.h" // TODO: remove if not used elsewhere

// Custom sensor includes
extern "C" {
#include "driver/i2c.h"
#include "i2c_bus.h"
#include "tca9548a.h"
}

static const char *TAG = "robot_task";

// Robot task state definitions (copy from task_manager.h to avoid circular
// import)
typedef enum {
  ROBOT_STATE_INIT,
  ROBOT_STATE_I2C_SETUP,
  ROBOT_STATE_SCAN_DEVICES,
  ROBOT_STATE_INIT_MPU9250,
  ROBOT_STATE_CALIBRATING,
  ROBOT_STATE_INIT_MOTORS,
  ROBOT_STATE_RUNNING,
  ROBOT_STATE_ERROR,
  ROBOT_STATE_SENSOR_ERROR,
  ROBOT_STATE_MOTOR_ERROR
} robot_state_t;

// Robot state callback function type
typedef void (*robot_state_callback_t)(robot_state_t state);

// Parameter structure for robot task
typedef struct {
  robot_state_callback_t state_callback;
} robot_task_params_t;

// Global callback pointer
static robot_state_callback_t g_state_callback = NULL;

// Helper function to set robot state
static void set_robot_state(robot_state_t state) {
  if (g_state_callback != NULL) {
    g_state_callback(state);
  }
}

// Shared I2C bus handle for all sensors
static i2c_bus_handle_t shared_i2c_bus = nullptr;
static tca9548a_handle_t *mux_handle = nullptr;

// AS5600 sensor instances
CAS5600 as5600_left = CAS5600();
CAS5600 as5600_right = CAS5600();
MPU9250 *mpu9250;

BLDCMotor motorLeft = BLDCMotor(7, 10, 150);
BLDCDriver3PWM driverLeft = BLDCDriver3PWM(
    LEFT_MOTOR_CHANNEL_A, LEFT_MOTOR_CHANNEL_B, LEFT_MOTOR_CHANNEL_C);

BLDCMotor motorRight = BLDCMotor(7, 10, 150);
BLDCDriver3PWM driverRight = BLDCDriver3PWM(
    RIGHT_MOTOR_CHANNEL_A, RIGHT_MOTOR_CHANNEL_B, RIGHT_MOTOR_CHANNEL_C);

#define OFF_SET 11

PIDController pid_stab_L{DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, 100000,
                         100}; // High ramp = no rate limiting
PIDController pid_stab_R{DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, 100000, 100};
PIDController pid_pos{DEFAULT_KP2, DEFAULT_KI2, DEFAULT_KD2, 10000,
                      30}; // Lower ramp for smoother velocity changes
PIDController pid_dir{DEFAULT_KP3, DEFAULT_KI3, DEFAULT_KD3, 1000,
                      1}; // Even lower ramp for direction control

// Optional filters (tune time constants as needed)
LowPassFilter lpf_error_pos{0.1f}; // Filter for position error used by pid_pos
LowPassFilter lpf_error_dir{
    0.05f}; // Filter for direction error used by pid_dir

// Sensor data structure
static struct {
  // MPU9250 data
  mpu9250_angles_t angles;

  // AS5600 data
  float as5600_left_angle;
  float as5600_right_angle;

  bool data_ready;
} sensor_data = {};

static struct {
  // AS5600 data
  float motor_left_speed;
  float motor_right_speed;

  bool data_ready;
} motor_data = {};

static SemaphoreHandle_t data_mutex = nullptr;  // mutex for sensor_data
static SemaphoreHandle_t motor_mutex = nullptr; // mutex for motor_data
static TaskHandle_t update_task_handle = nullptr;
static TaskHandle_t display_task_handle = nullptr;
static TaskHandle_t boot_btn_task_handle = nullptr;

// PID parameter protection
static SemaphoreHandle_t pid_param_mutex = nullptr;

// Optional runtime-tunable variables
static volatile float g_target_leg_command = 0.0f;
static volatile bool g_logging_enabled =
    false; // display task prints only when enabled
// Removed unused motion command variable; movement functions retained as no-ops
static volatile bool g_bldc_enabled = false; // global enable for BLDC output

static volatile float target_angle_left = 0.0f;
static volatile float target_angle_right = 0.0f;

static volatile float base_left = 0.0f;
static volatile float base_right = 0.0f;

// Minimal synchronization for target angle pair (avoid inconsistent reads)
static portMUX_TYPE target_mux = portMUX_INITIALIZER_UNLOCKED;
static inline void targets_set(float l, float r) {
  portENTER_CRITICAL(&target_mux);
  target_angle_left = l;
  target_angle_right = r;
  portEXIT_CRITICAL(&target_mux);
}
static inline void targets_get(float *l, float *r) {
  portENTER_CRITICAL(&target_mux);
  *l = target_angle_left;
  *r = target_angle_right;
  portEXIT_CRITICAL(&target_mux);
}

extern "C" bool robot_pid1_set_kp(float v) {
  if (pid_param_mutex &&
      xSemaphoreTake(pid_param_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    pid_stab_L.P = v;
    pid_stab_R.P = v;
    xSemaphoreGive(pid_param_mutex);
    return true;
  }
  return false;
}
extern "C" bool robot_pid1_set_ki(float v) {
  if (pid_param_mutex &&
      xSemaphoreTake(pid_param_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    pid_stab_L.I = v;
    pid_stab_R.I = v;
    xSemaphoreGive(pid_param_mutex);
    return true;
  }
  return false;
}
extern "C" bool robot_pid1_set_kd(float v) {
  if (pid_param_mutex &&
      xSemaphoreTake(pid_param_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    pid_stab_L.D = v;
    pid_stab_R.D = v;
    xSemaphoreGive(pid_param_mutex);
    return true;
  }
  return false;
}

extern "C" bool robot_pid2_set_kp(float v) {
  if (pid_param_mutex &&
      xSemaphoreTake(pid_param_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    pid_pos.P = v;
    xSemaphoreGive(pid_param_mutex);
    return true;
  }
  return false;
}
extern "C" bool robot_pid2_set_ki(float v) {
  if (pid_param_mutex &&
      xSemaphoreTake(pid_param_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    pid_pos.I = v;
    xSemaphoreGive(pid_param_mutex);
    return true;
  }
  return false;
}
extern "C" bool robot_pid2_set_kd(float v) {
  if (pid_param_mutex &&
      xSemaphoreTake(pid_param_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    pid_pos.D = v;
    xSemaphoreGive(pid_param_mutex);
    return true;
  }
  return false;
}

extern "C" bool robot_pid3_set_kp(float v) {
  if (pid_param_mutex &&
      xSemaphoreTake(pid_param_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    pid_dir.P = v;
    xSemaphoreGive(pid_param_mutex);
    return true;
  }
  return false;
}
extern "C" bool robot_pid3_set_ki(float v) {
  if (pid_param_mutex &&
      xSemaphoreTake(pid_param_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    pid_dir.I = v;
    xSemaphoreGive(pid_param_mutex);
    return true;
  }
  return false;
}
extern "C" bool robot_pid3_set_kd(float v) {
  if (pid_param_mutex &&
      xSemaphoreTake(pid_param_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    pid_dir.D = v;
    xSemaphoreGive(pid_param_mutex);
    return true;
  }
  return false;
}

extern "C" void robot_set_target_leg_command(float v) {
  g_target_leg_command = v;
}
extern "C" float robot_get_target_leg_command(void) {
  return g_target_leg_command;
}
extern "C" bool robot_toggle_logging(void) {
  g_logging_enabled = !g_logging_enabled;
  return g_logging_enabled;
}
extern "C" void robot_move_forward(void) { /* no-op: motion command not used */
}
extern "C" void robot_move_backward(void) { /* no-op: motion command not used */
}
extern "C" void robot_stop(void) { /* no-op: motion command not used */
}
extern "C" void robot_motor_start(void) {
  g_bldc_enabled = true;
  if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (sensor_data.data_ready) {
      float l = sensor_data.as5600_left_angle;
      float r = sensor_data.as5600_right_angle;
      xSemaphoreGive(data_mutex);
      targets_set(l, r);
      return;
    }
    xSemaphoreGive(data_mutex);
  }
}
extern "C" void robot_motor_stop(void) { g_bldc_enabled = false; }
extern "C" bool robot_motor_toggle(void) {
  g_bldc_enabled = !g_bldc_enabled;
  return g_bldc_enabled;
}

// Function to update target angles from servo task
extern "C" void robot_update_target_angles_from_servo(float left_offset, float right_offset) {
  // Get current sensor readings as base
  base_left = left_offset * M_PI / 180.0f;
  base_right = right_offset * M_PI / 180.0f;
}

// Task: Monitor BOOT button (GPIO0) and enable BLDC when pressed
static void boot_button_task(void *pv) {
  const gpio_num_t BOOT_BTN = GPIO_NUM_0; // ESP32-S3 BOOT button
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL << BOOT_BTN);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE; // BOOT is typically pulled up,
                                           // active-low when pressed
  gpio_config(&io_conf);

  bool prev_pressed = false;
  ESP_LOGI(TAG, "BOOT button task started (GPIO0)");
  while (1) {
    int level = gpio_get_level(BOOT_BTN);
    bool pressed = (level == 0);
    if (pressed && !prev_pressed) {
      robot_motor_start();
      ESP_LOGI(TAG, "BOOT pressed -> BLDC enabled");
    }
    prev_pressed = pressed;
    vTaskDelay(pdMS_TO_TICKS(50)); // debounce/polling interval
  }
}

// Function to initialize I2C bus and multiplexer
static esp_err_t init_i2c_system(void) {
  set_robot_state(ROBOT_STATE_I2C_SETUP);

  // Create shared I2C bus for all sensors
  i2c_config_t i2c_conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master =
          {
              .clk_speed = I2C_MASTER_FREQ_HZ,
          },
      .clk_flags = 0,
  };

  shared_i2c_bus = i2c_bus_create(I2C_NUM_0, &i2c_conf);
  if (!shared_i2c_bus) {
    ESP_LOGE(TAG, "Failed to create shared I2C bus");
    set_robot_state(ROBOT_STATE_ERROR);
    return ESP_FAIL;
  }
  ESP_LOGD(TAG, "Shared I2C bus created successfully");

  // Set bus for AS5600 sensors
  as5600_left.setBus(shared_i2c_bus);
  as5600_right.setBus(shared_i2c_bus);

  // Initialize TCA9548A multiplexer
  mux_handle = tca9548a_init(shared_i2c_bus, TCA9548A_DEFAULT_ADDR);
  if (!mux_handle) {
    ESP_LOGE(TAG, "Failed to initialize TCA9548A");
    set_robot_state(ROBOT_STATE_ERROR);
    return ESP_FAIL;
  }
  ESP_LOGD(TAG, "TCA9548A multiplexer initialized");

  return ESP_OK;
}

// Function to scan I2C channels for debugging
static void scan_i2c_channels(void) {
  set_robot_state(ROBOT_STATE_SCAN_DEVICES);
  ESP_LOGD(TAG, "Scanning all TCA9548A channels for devices...");
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
          ESP_LOGD(TAG, "  Device found at address 0x%02X", addr);
        }
      }
    } else {
      ESP_LOGE(TAG, "  Failed to select channel %d", ch);
    }
  }
}

// Function to initialize MPU9250 sensor
static esp_err_t init_mpu9250(void) {
  set_robot_state(ROBOT_STATE_INIT_MPU9250);
  ESP_LOGI(TAG, "Initializing MPU9250 on channel 1...");
  esp_err_t ret = tca9548a_select_channel(mux_handle, TCA9548A_CHANNEL_1);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to select MPU9250 channel: %s", esp_err_to_name(ret));
    set_robot_state(ROBOT_STATE_SENSOR_ERROR);
    return ret;
  }

  mpu9250 = new MPU9250(shared_i2c_bus);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize MPU9250: %s", esp_err_to_name(ret));
    set_robot_state(ROBOT_STATE_SENSOR_ERROR);
    return ret;
  }
  ESP_LOGD(TAG, "MPU9250 initialized on channel 1");

  // Calibrate MPU9250 gyroscope
  set_robot_state(ROBOT_STATE_CALIBRATING);
  ESP_LOGD(TAG, "Starting gyroscope calibration...");
  ret = mpu9250->begin();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "MPU9250 gyroscope calibration failed: %s",
             esp_err_to_name(ret));
    set_robot_state(ROBOT_STATE_SENSOR_ERROR);
  }

  return ESP_OK;
}

// Function to initialize motor and sensor pair
static esp_err_t init_motor_sensor_pair(int channel, CAS5600 &sensor,
                                        BLDCMotor &motor,
                                        BLDCDriver3PWM &driver,
                                        const char *name) {
  set_robot_state(ROBOT_STATE_INIT_MOTORS);
  esp_err_t ret = tca9548a_select_channel(mux_handle, channel);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to select %s channel: %s", name,
             esp_err_to_name(ret));
    set_robot_state(ROBOT_STATE_MOTOR_ERROR);
    return ret;
  }

  sensor.init();
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);
  motor.controller = MotionControlType::torque;
  motor.voltage_limit = 16; // Reduced from 12V to prevent MCPWM errors
  motor.init();
  motor.initFOC();

  ESP_LOGD(TAG, "%s sensor and motor initialized on channel %d", name, channel);
  return ESP_OK;
}

// Function to read all sensor data
static void read_sensor_data(void) {

  float left = 0, right = 0;

  // Read MPU9250 data (channel 1)
  esp_err_t ret = tca9548a_select_channel(mux_handle, TCA9548A_CHANNEL_1);
  if (ret == ESP_OK) {
    mpu9250->readSensor();
  }

  // Read AS5600 right sensor (same channel as MPU9250)
  right = as5600_right.getAngle();

  // Read AS5600 left sensor (channel 0)
  ret = tca9548a_select_channel(mux_handle, TCA9548A_CHANNEL_0);
  if (ret == ESP_OK) {
    left = as5600_left.getAngle();
  }

  // Update shared data structure with mutex protection
  if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
    // MPU9250 data
    sensor_data.angles.x = mpu9250->getAngleX();
    sensor_data.angles.y = mpu9250->getAngleY();
    sensor_data.angles.z = mpu9250->getAngleZ();

    // AS5600 data
    sensor_data.as5600_left_angle = left;
    sensor_data.as5600_right_angle = right;

    sensor_data.data_ready = true;
    xSemaphoreGive(data_mutex);
  }
}

static void caculate_pid() {

  // Local copies of sensor data
  mpu9250_angles_t angles = {};
  float left_angle = 0.0f;
  float right_angle = 0.0f;
  float target_l = 0.0f, target_r = 0.0f;
  targets_get(&target_l, &target_r);
  left_angle = target_l;
  right_angle = target_r;
  bool data_ready = false;

  /* Copy data with mutex protection */
  if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (sensor_data.data_ready) {
      angles = sensor_data.angles;
      left_angle = sensor_data.as5600_left_angle;
      right_angle = sensor_data.as5600_right_angle;
    }
    xSemaphoreGive(data_mutex);
  }

  if (pid_param_mutex &&
      xSemaphoreTake(pid_param_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {

    float error_pos_left = left_angle - (target_l + base_left);
    float error_pos_right = -(right_angle - (target_r + base_right));

    float error_pos = (error_pos_left + error_pos_right) / 2;
    // Low-pass filter the position error to reduce oscillations/noise
    float error_pos_f = lpf_error_pos(error_pos);
    float output_pos = pid_pos(error_pos_f);

    float error_dir = error_pos_left - error_pos_right;
    float error_dir_f = lpf_error_dir(error_dir);
    float output_dir = pid_dir(error_dir_f);

    float error_left = -(angles.y - OFF_SET) + output_pos;
    float error_right = -(angles.y - OFF_SET) + output_pos;

    float output_l = pid_stab_L(error_left) - output_dir;
    float output_r = pid_stab_R(error_right) + output_dir;

    // ESP_LOGI(TAG, "error_left [%6.2f] erorr_right[%6.2f] error-L[%6.2f°]
    // error-R[%6.2f°] error_pos-L[%6.2f°] output_pos-R[%6.2f°]",
    //                error_left, error_right, error_pos_left, error_pos_right,
    //                error_pos, output_pos);

    if (xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      motor_data.motor_left_speed = output_l;
      motor_data.motor_right_speed = output_r;
      motor_data.data_ready = true;
      xSemaphoreGive(motor_mutex);
    }
    xSemaphoreGive(pid_param_mutex);
  }
}

extern "C" void robot_update_task(void *pvParameters) {
  ESP_LOGI(TAG, "Robot update task started (1ms interval)");
  set_robot_state(ROBOT_STATE_INIT);

  vTaskDelay(pdMS_TO_TICKS(100)); // 1000Hz update rate
  // Initialize I2C system and multiplexer
  if (init_i2c_system() != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize I2C system");
    set_robot_state(ROBOT_STATE_ERROR);
    vTaskDelete(NULL);
    return;
  }

  // Optional heavy scan disabled to minimize startup time and logs
  // scan_i2c_channels();

  // Initialize MPU9250 sensor
  if (init_mpu9250() != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize MPU9250");
    set_robot_state(ROBOT_STATE_SENSOR_ERROR);
    vTaskDelete(NULL);
    return;
  }

  vTaskDelay(pdMS_TO_TICKS(100)); // 1000Hz update rate
  // Initialize right motor and sensor pair
  if (init_motor_sensor_pair(RIGHT_CHANNEL, as5600_right, motorRight,
                             driverRight, "AS5600 right") != ESP_OK) {
    set_robot_state(ROBOT_STATE_MOTOR_ERROR);
    vTaskDelete(NULL);
    return;
  }

  // Initialize left motor and sensor pair
  if (init_motor_sensor_pair(LEFT_CHANNEL, as5600_left, motorLeft, driverLeft,
                             "AS5600 left") != ESP_OK) {
    set_robot_state(ROBOT_STATE_MOTOR_ERROR);
    vTaskDelete(NULL);
    return;
  }

  ESP_LOGD(TAG, "Starting high-frequency sensor data acquisition...");
  set_robot_state(ROBOT_STATE_RUNNING);

  read_sensor_data();
  // Seed target angles to current readings atomically
  if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    float l = sensor_data.as5600_left_angle;
    float r = sensor_data.as5600_right_angle;
    xSemaphoreGive(data_mutex);
    targets_set(l, r);
  }

  // int speed = 0;
  // const int speed_step = 2;
  // const int speed_max = 300;
  // int loop_count = 0;
  //
  // while (1) {
  //   // Gửi tốc độ cho động cơ (trái/phải giống nhau)
  //   if (xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
  //       esp_err_t ret = tca9548a_select_channel(mux_handle, LEFT_CHANNEL);
  //       if (ret == ESP_OK) {
  //           motorLeft.loopFOC();
  //           motorLeft.move((float)speed);
  //       }
  //       ret = tca9548a_select_channel(mux_handle, RIGHT_CHANNEL);
  //       if (ret == ESP_OK) {
  //           motorRight.loopFOC();
  //           motorRight.move((float)speed);
  //       }
  //       xSemaphoreGive(motor_mutex);
  //   }
  //   // vTaskDelay(pdMS_TO_TICKS(1)); // 1 giây
  //
  //   loop_count++;
  //   if (loop_count >= 100) {
  //       loop_count = 0;
  //       speed += speed_step;
  //       ESP_LOGI("MOTOR", "Current speed: %.2f (loop %d)", (float)speed, loop_count);
  //       if (speed > speed_max) speed = 0; // reset về 0 nếu vượt max
  //   }
  // }

  // Main sensor reading loop
  while (1) {
    read_sensor_data();
    caculate_pid();
    if (g_bldc_enabled &&
        xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (motor_data.data_ready) {
        float left_cmd = motor_data.motor_left_speed;
        float right_cmd = motor_data.motor_right_speed;
        esp_err_t ret = tca9548a_select_channel(mux_handle, LEFT_CHANNEL);
        if (ret == ESP_OK) {
          motorLeft.loopFOC();
          motorLeft.move(left_cmd);
        }
        ret = tca9548a_select_channel(mux_handle, RIGHT_CHANNEL);
        if (ret == ESP_OK) {
          motorRight.loopFOC();
          motorRight.move(right_cmd);
        }
        motor_data.data_ready = false;
      }
      xSemaphoreGive(motor_mutex);
    }

    // vTaskDelay(pdMS_TO_TICKS(1)); // 1000Hz update rate
  }
}

extern "C" void mpu9250_display_task(void *pvParameters) {
  ESP_LOGD(TAG, "Sensor display task started (100ms interval)");

  // Local copies of sensor data
  mpu9250_angles_t angles;
  float left_angle, right_angle;
  bool data_ready = false;

  while (1) {
    /* Copy data with mutex protection */
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (sensor_data.data_ready) {
        angles = sensor_data.angles;
        left_angle = sensor_data.as5600_left_angle;
        right_angle = sensor_data.as5600_right_angle;
        data_ready = true;
      }
      xSemaphoreGive(data_mutex);
    }

    /* Display all 3 sensors in one log line (only when logging enabled) */
    if (data_ready && g_logging_enabled) {
      float t_l = 0.0f, t_r = 0.0f;
      targets_get(&t_l, &t_r);
      ESP_LOGI(TAG,
               "MPU9250[X:%6.2f° Y:%6.2f° Z:%6.2f°] AS5600-L[%6.2f°] "
               "AS5600-R[%6.2f°] T-L[%6.2f°] T-R[%6.2f°]",
               angles.x, angles.y, angles.z, left_angle, right_angle, t_l, t_r);
      data_ready = false;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

extern "C" void robot_task(void *pvParameters) {
  ESP_LOGD(TAG, "Robot main task started - creating update and display tasks");

  // Lấy callback từ parameters
  robot_task_params_t *params = (robot_task_params_t *)pvParameters;
  if (params != NULL && params->state_callback != NULL) {
    g_state_callback = params->state_callback;
    ESP_LOGD(TAG, "Robot state callback registered successfully");
  } else {
    ESP_LOGW(TAG, "No state callback provided - LED status will not work");
  }

  data_mutex = xSemaphoreCreateMutex();
  motor_mutex = xSemaphoreCreateMutex();
  pid_param_mutex = xSemaphoreCreateMutex();
  if (data_mutex == nullptr || motor_mutex == nullptr) {
    ESP_LOGE(TAG, "Failed to create mutex");
    set_robot_state(ROBOT_STATE_ERROR);
    vTaskDelete(NULL);
    return;
  }
  BaseType_t ret = xTaskCreate(robot_update_task, "robot_update", 4096, NULL,
                               configMAX_PRIORITIES - 1, &update_task_handle);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create Robot update task");
    set_robot_state(ROBOT_STATE_ERROR);
    vSemaphoreDelete(data_mutex);
    vSemaphoreDelete(motor_mutex);
    vTaskDelete(NULL);
    return;
  }
  ret = xTaskCreate(mpu9250_display_task, "mpu9250_display", 4096, NULL,
                    tskIDLE_PRIORITY + 2, &display_task_handle);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create MPU9250 display task");
    set_robot_state(ROBOT_STATE_ERROR);
    if (update_task_handle != nullptr) {
      vTaskDelete(update_task_handle);
    }
    vSemaphoreDelete(data_mutex);
    vSemaphoreDelete(motor_mutex);
    vTaskDelete(NULL);
    return;
  }
  // Start BOOT button monitor task
  ret = xTaskCreate(boot_button_task, "boot_button", 4096, NULL,
                    tskIDLE_PRIORITY + 1, &boot_btn_task_handle);
  if (ret != pdPASS) {
    ESP_LOGW(TAG, "Failed to create BOOT button task");
  }
  ESP_LOGD(TAG, "Robot tasks created successfully");
  vTaskDelete(NULL);
}
