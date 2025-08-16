# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a complete Furuta pendulum (rotary pendulum) control system implemented for ESP32 using ESP-IDF framework. The system implements advanced control algorithms including PID control, energy-based swing-up control, and real-time physics simulation for controlling a classic Furuta pendulum - an inverted pendulum mounted on a rotating arm.

## Development Commands

### Building and Flashing
```bash
# Initialize ESP-IDF environment
init_idf

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
- **Motor Control**: GPIO 19 (A1), GPIO 21 (B1), GPIO 22 (A2), GPIO 23 (B2)
- **Motor PWM**: GPIO 26 (PWM1), GPIO 27 (PWM2)
- **Pendulum Encoder (A38)**: GPIO 32 (Channel A), GPIO 33 (Channel B)
- **Arm Encoder (HW-040)**: GPIO 4 (Channel A), GPIO 5 (Channel B)
- **Power Supply**: 12V for A38 encoder, 3.3V/5V for HW-040, 3.3V logic levels for ESP32

### Hardware Components
- **Motor Driver**: Monster Moto Mini stepper motor driver
- **Pendulum Encoder**: A38S6-400-2-24 rotary encoder (400 PPR, 1600 counts/rev with quadrature)
- **Arm Encoder**: HW-040 rotary encoder (20 PPR, 80 counts/rev with quadrature)
- **Motor**: Stepper motor for arm rotation (with encoder feedback)
- **Pendulum**: Physical pendulum mounted on rotating arm

### PWM Configuration
- **Timer**: LEDC_TIMER_0
- **Channels**: LEDC_CHANNEL_0, LEDC_CHANNEL_1
- **Frequency**: 10kHz
- **Resolution**: 8-bit (0-255 duty cycle)
- **Mode**: High-speed mode

## Software Architecture

### Component Structure
```
components/
├── motor/          # Stepper motor control with Monster Moto Mini
├── encoder/        # A38 pendulum encoder interface with interrupt-driven position tracking
├── hw040_encoder/  # HW-040 arm encoder interface with interrupt-driven position tracking
├── control/        # PID controllers and control algorithms (furuta_control)
├── physics/        # Physics simulation and state estimation (furuta_physics)
└── sensors/        # Sensor fusion and data filtering (furuta_sensors)
```

### Main Application (`main/pedulo_furuta.c`)
- **Control Loop**: 100 Hz real-time control task
- **System Initialization**: Hardware and component setup
- **Calibration**: Automatic sensor offset calibration
- **Safety Systems**: Emergency stop and safety monitoring

### Control System Features
- **Swing-Up Control**: Energy-based control to bring pendulum to upright position
- **Balance Control**: PID-based balancing when pendulum is near vertical
- **Mode Switching**: Automatic mode transitions based on pendulum state
- **Safety Limits**: Velocity and angle limits with emergency stop

### Key Components

#### Motor Component (`components/motor/`)
- Stepper motor control with 4-step sequence
- PWM-based torque control
- Velocity control interface for arm rotation
- Emergency stop functionality
- Works with HW-040 encoder feedback for closed-loop control

#### Encoder Component (`components/encoder/`)
- Interrupt-driven quadrature decoding for pendulum angle (A38)
- Real-time pendulum position and velocity calculation
- Thread-safe design with mutex protection
- 400 PPR resolution (1600 counts/rev)
- Measures pendulum angle: 0° = hanging down, 180° = upright

#### HW-040 Encoder Component (`components/hw040_encoder/`)
- Interrupt-driven quadrature decoding for arm angle
- Real-time arm position and velocity calculation
- Thread-safe design with mutex protection
- 20 PPR resolution (80 counts/rev)
- Measures arm rotation angle for closed-loop control

#### Control Component (`components/control/`)
- PID controllers for arm position and pendulum balance
- Energy-based swing-up algorithm
- Safety checking and mode determination
- Configurable control parameters

#### Physics Component (`components/physics/`)
- Real-time state estimation
- Velocity calculation with history
- Energy calculations
- Angle normalization and utilities

#### Sensors Component (`components/sensors/`)
- Sensor fusion and data validation
- Velocity filtering and smoothing
- Calibration and offset management
- Data quality monitoring

## Control Modes

1. **IDLE**: System at rest, no control active
2. **SWING_UP**: Energy-based control to swing pendulum upward
3. **BALANCE**: PID control to maintain pendulum in upright position
4. **MANUAL**: Manual control mode (for testing)
5. **EMERGENCY_STOP**: Safety mode with all motors stopped

## Development Notes

### ESP-IDF Framework
- **Version**: ESP-IDF v5.5.0
- **Target**: ESP32
- **Build System**: CMake with component-based architecture
- **Real-time**: FreeRTOS with 100 Hz control loop

### Control Parameters
- **Control Frequency**: 100 Hz
- **PID Gains**: Tunable for arm position and pendulum balance
- **Safety Limits**: Configurable velocity and angle limits
- **Physical Parameters**: Arm length, pendulum mass, etc.

### Important Notes
- System requires proper calibration before operation
- Pendulum should start in hanging-down position
- Emergency stop functionality always available
- Real-time control requires proper task priorities

### Hardware Dependencies
- ESP32 development board with sufficient GPIO pins
- 12V power supply for encoder
- Monster Moto Mini stepper motor driver
- A38S6-400-2-24 rotary encoder with proper wiring