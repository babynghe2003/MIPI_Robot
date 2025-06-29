# Audio Module

This module will contain audio processing and I2S microphone functionality.

## Planned Features:
- I2S microphone input
- Audio preprocessing for ESP-SR
- Audio playback through I2S DAC/amplifier
- Voice activity detection

## Files to add:
- `i2s_mic.c/h` - I2S microphone driver
- `audio_preprocessor.c/h` - Audio signal preprocessing
- `audio_task.c` - Audio processing task
- `voice_commands.c/h` - Voice command recognition integration

## Configuration:
Update `app_config.h` to enable:
```c
#define MODULE_AUDIO_ENABLED        1
```

## ESP-SR Integration:
This module will integrate with ESP-SR for voice command recognition once the audio pipeline is established.
