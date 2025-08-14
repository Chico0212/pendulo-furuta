# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Furuta pendulum (rotary pendulum) control system implemented for ESP32 using ESP-IDF framework. The project controls stepper motors through GPIO pins and PWM channels to implement mechanical control of a Furuta pendulum - a classic control systems example with a rotating arm and pendulum.

## Development Commands

### Building and Flashing
```bash
# Build the project
idf.py build

# Flash to ESP32 device
idf.py flash

# Monitor serial output
idf.py monitor

# Build, flash, and monitor in one command
idf.py build flash monitor

# Clean build artifacts
idf.py clean
```

### Configuration
```bash
# Open menuconfig for ESP-IDF configuration
idf.py menuconfig
```

## Hardware Architecture

### GPIO Pin Configuration
- **Motor 1 Control**: GPIO 19 (A1), GPIO 21 (B1)
- **Motor 2 Control**: GPIO 22 (A2), GPIO 23 (B2)  
- **PWM Channels**: GPIO 26 (PWM1), GPIO 27 (PWM2)

### Motor Control System
The system uses a 4-step stepper motor control sequence with precise timing:
1. **Step 1**: A+ B+ (17ms delay)
2. **Step 2**: A- B+ (17ms delay)  
3. **Step 3**: A- B- (17ms delay)
4. **Step 4**: A+ B- (17ms delay)

### PWM Configuration
- **Timer**: LEDC_TIMER_0
- **Channels**: LEDC_CHANNEL_0, LEDC_CHANNEL_1
- **Frequency**: 36kHz
- **Resolution**: 8-bit (0-255 duty cycle)
- **Mode**: High-speed mode

## Code Architecture

### Main Components
- **`main/pedulo_furuta.c`**: Single main source file containing all control logic
- **Hardware Initialization**: GPIO configuration for motor control pins
- **PWM Setup**: LEDC timer and channel configuration for motor speed control
- **Motor Control**: `clock_wise_spin()` function implementing 4-step stepper sequence

### Key Functions
- **`app_main()`**: Main application entry point, initializes hardware and runs motor sequence
- **`ledc_setup()`**: Configures PWM timers and channels for motor speed control
- **`clock_wise_spin()`**: Executes one complete 4-step clockwise rotation sequence

## Development Notes

### ESP-IDF Framework
This project uses ESP-IDF v5.5.0 with standard ESP32 target configuration. The build system uses CMake with component-based architecture.

### FreeRTOS Integration
Motor timing uses `vTaskDelay(pdMS_TO_TICKS(17))` for precise 17ms delays between stepper motor steps.

### Hardware Dependencies
Project assumes specific ESP32 development board with GPIO pins available for motor control and PWM output.