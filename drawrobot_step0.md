# DrawRobot Step 0: Foundation & Project Setup

> **Establish the project structure, hardware configuration, and build system**

---

## 📋 Objectives

By the end of this step, you will have:
- ✅ Directory structure for C++ code, headers, and web assets
- ✅ `Config.h` with all hardware pinouts and constants
- ✅ `platformio.ini` configured for ESP32 with real/sim modes
- ✅ Build system working (PlatformIO)
- ✅ Basic `main.cpp` skeleton with WiFi setup
- ✅ Understanding of the project layout

---

## 🏗️ Project Directory Structure

```
DrawRobot/
├── platformio.ini                 # Build config, environment definitions
├── src/
│   ├── main.cpp                   # Entry point, initialization, main loop
│   ├── IMotor.cpp                 # (Empty for now, interface only)
│   ├── DCMotor.cpp                # Motor control implementation
│   ├── VirtualStepper.cpp         # Simulation motor
│   ├── DriveTrain.cpp             # Differential drive kinematics
│   ├── Navigation.cpp             # IMU sensor reading
│   ├── PoseEstimator.cpp          # Odometry + IMU fusion
│   ├── MotionController.cpp       # PID-based motion control
│   ├── PathPlanner.cpp            # Shape/text generation
│   ├── CommsManager.cpp           # WebSocket + JSON telemetry
│   ├── ConfigManager.cpp          # NVS persistent storage
│   ├── BatteryMonitor.cpp         # Battery voltage monitoring
│   └── BluetoothManager.cpp       # Bluetooth debug bridge
├── include/
│   ├── Config.h                   # Hardware constants (pins, physics)
│   ├── IMotor.h                   # Motor interface (pure virtual)
│   ├── DCMotor.h                  # Real motor driver
│   ├── VirtualStepper.h           # Simulated motor
│   ├── DriveTrain.h               # Differential drive controller
│   ├── Navigation.h               # IMU sensor interface
│   ├── PoseEstimator.h            # Position & heading tracker
│   ├── MotionController.h         # Waypoint follower
│   ├── PathPlanner.h              # Geometry & text generator
│   ├── CommsManager.h             # WebSocket server
│   ├── ConfigManager.h            # Parameter storage
│   ├── BatteryMonitor.h           # Battery monitoring
│   └── BluetoothManager.h         # BLE debugging
├── data/
│   ├── index.html                 # Web dashboard
│   ├── style.css                  # Dashboard styling
│   └── script.js                  # Dashboard interactivity
└── docs/
    ├── drawrobot.md               # This plan
    └── drawrobot_step*.md         # Each step

```

---

## 🔧 Step 0 Implementation

### 1. **Create Config.h** (Hardware Configuration)

```cpp
// include/Config.h
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <cmath>

// ============================================================================
// FIRMWARE VERSION & BUILD INFO
// ============================================================================
constexpr char FirmwareVersion[] = "1.0.0";

// ============================================================================
// PHYSICAL ROBOT PARAMETERS (from travail.pdf)
// ============================================================================
constexpr float WheelDiameterMm = 90.0f;      // Drive wheel diameter
constexpr float WheelBaseMm = 83.0f;          // Distance between wheel centers (track width)
constexpr float PenOffsetMm = 130.0f;         // Distance from axle to pen tip (front)
constexpr int StepsPerRev = 1070;             // Encoder steps per wheel revolution
constexpr float MaxLinearSpeedMmS = 120.0f;   // Maximum forward/backward speed
constexpr float MaxAngularSpeedRadS = 2.9f;   // Maximum rotation speed (rad/s)
constexpr int MotorPwmDeadband = 151;         // Minimum PWM to overcome static friction

// Derived constants (computed at compile time)
constexpr float WheelCircumferenceMm = WheelDiameterMm * M_PI;
constexpr float MmPerStep = WheelCircumferenceMm / StepsPerRev;
constexpr float RadiusFromWheelMm = WheelBaseMm / 2.0f;

// ============================================================================
// MOTOR CONTROL - LEFT MOTOR (from pin.pdf)
// ============================================================================
constexpr int PinMotorLeftEn = 4;      // PWM enable (0-255)
constexpr int PinMotorLeftIn1 = 17;    // Direction control
constexpr int PinMotorLeftIn2 = 16;    // Direction control

// ============================================================================
// MOTOR CONTROL - RIGHT MOTOR (from pin.pdf)
// ============================================================================
constexpr int PinMotorRightEn = 23;    // PWM enable (0-255)
constexpr int PinMotorRightIn1 = 19;   // Direction control
constexpr int PinMotorRightIn2 = 18;   // Direction control

// ============================================================================
// ENCODER FEEDBACK - QUADRATURE A/B CHANNELS
// ============================================================================
constexpr int PinEncoderLeftA = 33;    // Left encoder phase A
constexpr int PinEncoderLeftB = 32;    // Left encoder phase B

constexpr int PinEncoderRightA = 27;   // Right encoder phase A
constexpr int PinEncoderRightB = 14;   // Right encoder phase B

// ============================================================================
// I2C BUS (IMU & Magnetometer Communication)
// ============================================================================
constexpr int PinI2cSda = 21;          // Serial data line
constexpr int PinI2cScl = 22;          // Serial clock line
constexpr int I2cFrequency = 400000;   // 400 kHz standard I2C speed

// IMU I2C addresses (auto-detected, from pin.pdf)
constexpr uint8_t AddrMpu9250 = 0x68;  // MPU9250 IMU (accel + gyro + mag)
constexpr uint8_t AddrBno055 = 0x28;   // BNO055 IMU (alternative)

// ============================================================================
// USER INTERFACE LEDS
// ============================================================================
constexpr int PinLedUser1 = 25;        // Status LED 1
constexpr int PinLedUser2 = 26;        // Status LED 2

// ============================================================================
// BATTERY MONITORING (ADC)
// ============================================================================
constexpr int PinBatteryAdc = 0;              // Battery voltage ADC input
constexpr float BatteryDividerRatio = 2.0f;   // Voltage divider: battery / ADC
constexpr float BatteryLowVoltage = 6.6f;     // Low battery threshold (4S LiPo min)
constexpr float BatteryFullVoltage = 16.8f;   // Full charge (4S LiPo max)

// ============================================================================
// PEN CONTROL (Third Wheel - Passive Contact)
// ============================================================================
// NOTE: Pen is a passive third wheel, not servo-controlled
// Maintains constant contact with drawing surface through gravity/spring
// No active control needed - pen always in contact when robot moves
constexpr int PinPenLift = 13;        // Optional: GPIO for mechanical lift actuator (if implemented)
                                       // Currently unused - pen is passive

// ============================================================================
// WIFI ACCESS POINT (from travail.pdf)
// ============================================================================
constexpr const char* WifiSsid = "RobotWifi";
constexpr const char* WifiPassword = "penbot123";
constexpr const char* WifiApIp = "192.168.4.1";
constexpr uint16_t WebsocketPort = 80;

// ============================================================================
// SYSTEM CONFIGURATION
// ============================================================================

// Define MODE_REAL to use actual hardware; otherwise, use simulation
enum class SystemRunMode {
    Sim,   // Virtual motors (no hardware required)
    Real   // Physical hardware with real sensors
};

#ifdef MODE_REAL
constexpr SystemRunMode SYSTEM_MODE = SystemRunMode::Real;
#else
constexpr SystemRunMode SYSTEM_MODE = SystemRunMode::Sim;
#endif

// ============================================================================
// CONTROL LOOP TIMING (milliseconds)
// ============================================================================
constexpr int LoopRateMsec = 10;           // 100 Hz main loop
constexpr int TelemetryRateMsec = 50;      // 20 Hz telemetry updates
constexpr int StatusLedToggleMsec = 500;   // Blink rate for status LED

// ============================================================================
// PID TUNING PARAMETERS (to be calibrated)
// ============================================================================
constexpr float PidLinearKp = 0.5f;        // Linear velocity proportional
constexpr float PidLinearKi = 0.05f;       // Linear velocity integral
constexpr float PidLinearKd = 0.1f;        // Linear velocity derivative

constexpr float PidAngularKp = 0.3f;       // Angular velocity proportional
constexpr float PidAngularKi = 0.02f;      // Angular velocity integral
constexpr float PidAngularKd = 0.05f;      // Angular velocity derivative

#endif // CONFIG_H
```

### 2. **Create platformio.ini** (Build Configuration)

```ini
; platformio.ini - PlatformIO configuration for ESP32
[platformio]
default_envs = sim
src_dir = src
include_dir = include
data_dir = data

[env]
; Common settings for all environments
platform = espressif32
board = esp32doit-devkit-v1
framework = arduino
monitor_speed = 115200
upload_speed = 921600

; Dependencies
lib_deps =
    ArduinoJson @ ^7.0.0
    ESPAsyncWebServer @ ^1.3.2
    ESP32Servo @ ^0.11.0

; Optimization
build_flags =
    -O2
    -Wall -Wextra -Wpedantic

; ============================================================================
; SIMULATION MODE (default - no hardware required)
; ============================================================================
[env:sim]
build_flags =
    ${env.build_flags}
    # MODE_REAL is NOT defined; uses VirtualStepper
    -DMODE_SIM

; ============================================================================
; REAL HARDWARE MODE (with actual motors, IMU, WiFi)
; ============================================================================
[env:real_robot]
build_flags =
    ${env.build_flags}
    -DMODE_REAL

; ============================================================================
; UPLOAD & MONITOR (for testing)
; ============================================================================
[env:real_robot]
upload_port = COM3          ; Change to your ESP32 serial port
monitor_port = COM3

; For Linux/Mac, use:
; upload_port = /dev/ttyUSB0
; monitor_port = /dev/ttyUSB0
```

### 3. **Create main.cpp Skeleton**

```cpp
// src/main.cpp
#include <Arduino.h>
#include <WiFi.h>
#include "Config.h"

// ============================================================================
// GLOBAL STATE
// ============================================================================
unsigned long lastStatusLedToggle = 0;
unsigned long lastTelemetryUpdate = 0;

// ============================================================================
// INITIALIZATION
// ============================================================================

void setupWifi() {
    Serial.println("[WiFi] Starting WiFi Access Point...");
    
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WifiSsid, WifiPassword);
    WiFi.softAPConfig(
        IPAddress(192, 168, 4, 1),      // IP
        IPAddress(192, 168, 4, 1),      // Gateway
        IPAddress(255, 255, 255, 0)     // Netmask
    );
    
    IPAddress ip = WiFi.softAPIP();
    Serial.print("[WiFi] AP IP: ");
    Serial.println(ip);
}

void setupGpio() {
    Serial.println("[GPIO] Initializing pins...");
    
    // Status LEDs
    pinMode(PinLedUser1, OUTPUT);
    pinMode(PinLedUser2, OUTPUT);
    digitalWrite(PinLedUser1, LOW);
    digitalWrite(PinLedUser2, LOW);
}

void setupSerial() {
    Serial.begin(115200);
    delay(500);  // Wait for serial to stabilize
    Serial.println("\n\n========================================");
    Serial.println("DrawRobot Firmware v1.0.0");
    Serial.println("========================================\n");
    
    // Print build info
#ifdef MODE_REAL
    Serial.println("[Boot] Mode: REAL HARDWARE");
#else
    Serial.println("[Boot] Mode: SIMULATION");
#endif
}

void setup() {
    setupSerial();
    setupGpio();
    setupWifi();
    
    Serial.println("[Boot] Initialization complete!");
    Serial.println("[Boot] Entering main loop...\n");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // Toggle status LED every 500ms
    if (millis() - lastStatusLedToggle > StatusLedToggleMsec) {
        digitalWrite(PinLedUser1, !digitalRead(PinLedUser1));
        lastStatusLedToggle = millis();
    }
    
    // Update telemetry every 50ms
    if (millis() - lastTelemetryUpdate > TelemetryRateMsec) {
        // TODO: Add telemetry updates in future steps
        lastTelemetryUpdate = millis();
    }
    
    // Keep loop timing consistent
    delay(LoopRateMsec);
}
```

---

## ✅ Verification Checklist

- [ ] Directory structure created as shown above
- [ ] `Config.h` contains all hardware constants from `pin.pdf`
- [ ] `platformio.ini` builds successfully:
  ```bash
  platformio run -e sim
  ```
- [ ] `main.cpp` compiles without errors
- [ ] Serial output shows startup messages and firmware version
- [ ] WiFi AP "RobotWifi" appears when powered (real hardware mode)
- [ ] Status LEDs blink at 1 Hz (sim mode, GPIO toggle)
- [ ] No compiler warnings about unused variables (yet)

---

## 🎯 Next Steps

Once this foundation is solid:
1. **Step 1**: Create `IMotor.h` interface
2. **Step 2**: Implement `DCMotor.cpp` for real motors
3. **Step 3**: Implement `VirtualStepper.cpp` for simulation

---

## 💡 Design Notes

- **Compile-time Constants**: All values in `Config.h` are `constexpr`, so they're computed at build time (zero runtime cost)
- **Two Build Modes**: The `MODE_REAL` flag switches between hardware and simulation without changing code
- **No Bloat**: Main loop is minimal; components added in subsequent steps
- **Const-Correctness**: Ready for `const` references in all functions

---

## 🔗 References

- `pin.pdf`: GPIO pinout and I2C addresses
- `travail.pdf`: Physical robot specs
- PlatformIO Documentation: https://docs.platformio.org/
