/* PID Console - minimal serial protocol to tune PID without flashing
 * Compatible with existing Python GUI single-letter commands.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "servo_control.h"

static const char *TAG = "pid_console";

// Exposed by robot_task.cpp
bool robot_pid1_set_kp(float v);
bool robot_pid1_set_ki(float v);
bool robot_pid1_set_kd(float v);
bool robot_pid2_set_kp(float v);
bool robot_pid2_set_ki(float v);
bool robot_pid2_set_kd(float v);
bool robot_pid3_set_kp(float v);
bool robot_pid3_set_ki(float v);
bool robot_pid3_set_kd(float v);
void robot_set_target_leg_command(float v);
float robot_get_target_leg_command(void);
bool robot_toggle_logging(void);
void robot_move_forward(void);
void robot_move_backward(void);
void robot_stop(void);
void robot_motor_start(void);
void robot_motor_stop(void);
bool robot_motor_toggle(void);

#define RX_BUF_SIZE 256

static void handle_line(const char *line)
{
    if (!line || !*line) return;
    char cmd = line[0];
    float val = 0.0f;
    if (strlen(line) > 1) {
        val = strtof(line + 1, NULL);
    }

    bool ok = true;
    switch (cmd) {
        case 'P': ok = robot_pid1_set_kp(val); break;
        case 'I': ok = robot_pid1_set_ki(val); break;
        case 'D': ok = robot_pid1_set_kd(val); break;
        case 'Q': ok = robot_pid2_set_kp(val); break;
        case 'W': ok = robot_pid2_set_ki(val); break;
        case 'R': ok = robot_pid2_set_kd(val); break;
        case 'A': ok = robot_pid3_set_kp(val); break;
        case 'B': ok = robot_pid3_set_ki(val); break;
        case 'C': ok = robot_pid3_set_kd(val); break;
        case 'V': robot_set_target_leg_command(val); break;
        case 'F': robot_move_forward(); break;
        case 'b': robot_move_backward(); break; // use lowercase to avoid collision with PID3 'B'
        case 'S': robot_stop(); break;
    case 'G': ok = robot_toggle_logging(); break; // Toggle sensor display logging
    case 'M': robot_motor_start(); break;        // Start/enable BLDC output
    case 'N': robot_motor_stop(); break;         // Stop/disable BLDC output
    case 'T': ok = robot_motor_toggle(); break;  // Toggle BLDC enable
    // New: servo target angle control (smoothly applied by servo task)
    case 'X': ok = (servo_set_target_idx(0, val) == ESP_OK); break; // Servo on GPIO 11
    case 'Y': ok = (servo_set_target_idx(1, val) == ESP_OK); break; // Servo on GPIO 4
        default:
            ESP_LOGW(TAG, "Unknown cmd: %c", cmd);
            ok = false;
            break;
    }
    ESP_LOGI(TAG, "cmd %c -> %s (%.6f)", cmd, ok?"OK":"ERR", val);
}

static void pid_console_task(void *arg)
{
    // Install USB Serial JTAG driver once before using it
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "USB Serial JTAG driver install failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    char buf[RX_BUF_SIZE];
    size_t idx = 0;
    ESP_LOGI(TAG, "PID console started on USB Serial JTAG (monitor /dev/ttyACM0)");
    while (1) {
        uint8_t byte;
        int len = usb_serial_jtag_read_bytes(&byte, 1, pdMS_TO_TICKS(20));
        if (len == 1) {
            if (byte == '\n' || byte == '\r') {
                buf[idx] = '\0';
                if (idx > 0) handle_line(buf);
                idx = 0;
            } else if (idx < RX_BUF_SIZE-1) {
                buf[idx++] = (char)byte;
            } else {
                idx = 0; // overflow, reset
            }
        }
    }
}

void pid_console_start(void)
{
    xTaskCreate(pid_console_task, "pid_console", 3072, NULL, 5, NULL);
}
