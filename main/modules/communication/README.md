# Communication Module

This module will contain wireless communication functionality.

## Planned Features:
- WiFi connectivity and management
- Bluetooth/BLE communication
- HTTP/MQTT client for IoT integration
- OTA (Over-The-Air) updates

## Files to add:
- `wifi_manager.c/h` - WiFi connection management
- `bluetooth_manager.c/h` - Bluetooth/BLE functionality
- `iot_client.c/h` - IoT cloud communication
- `ota_manager.c/h` - Over-the-air update functionality
- `communication_task.c` - Communication task coordination

## Configuration:
Update `app_config.h` to enable:
```c
#define MODULE_WIFI_ENABLED         1
```
