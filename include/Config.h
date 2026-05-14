#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <cmath>

namespace Config {
    // Firmware
    constexpr char FirmwareVersion[] = "1.0.0";

    // Physical Parameters
    constexpr float WHEEL_DIAMETER = 90.0f;
    constexpr float WHEEL_BASE = 83.0f;
    constexpr float PEN_OFFSET = 130.0f;
    constexpr int STEPS_PER_REV = 1070;
    constexpr float MM_PER_STEP = (WHEEL_DIAMETER * M_PI) / STEPS_PER_REV;
    constexpr float MM_PER_TICK = MM_PER_STEP; // Required by PoseEstimator

    // Motor Pins
    constexpr int PinMotorLeftEn = 4;
    constexpr int PinMotorLeftIn1 = 17;
    constexpr int PinMotorLeftIn2 = 16;
    constexpr int PinMotorRightEn = 23;
    constexpr int PinMotorRightIn1 = 19;
    constexpr int PinMotorRightIn2 = 18;

    // Encoder Pins
    constexpr int PinEncoderLeftA = 33;
    constexpr int PinEncoderLeftB = 32;
    constexpr int PinEncoderRightA = 27;
    constexpr int PinEncoderRightB = 14;

    // I2C
    constexpr int SDA = 21;
    constexpr int SCL = 22;
    constexpr int I2C_FREQ = 100000; // 100kHz (Standard Mode) for better noise immunity

    // IMU I2C addresses (Adafruit defaults)
    constexpr uint8_t AddrLsm6ds = 0x6A;   // Default for LSM6DSOX/LSM6DS33
    constexpr uint8_t AddrLis3mdl = 0x1E;  // LIS3MDL (alternate address, often used on breakout boards)

    // PWM Config
    constexpr int PWM_FREQ = 5000;
    constexpr int PWM_RES = 8;
    constexpr int DEADBAND = 151;

    // UI & Battery
    constexpr int PinLedUser1 = 25;
    constexpr int PinLedUser2 = 26;
    constexpr int PinBatteryAdc = 34;
    constexpr float BatteryDividerRatio = 2.0f;
    constexpr float BatteryLowVoltage = 6.6f;
    constexpr float BatteryFullVoltage = 16.8f;

    // WiFi
    constexpr const char* WifiSsid = "RobotWifi";
    constexpr const char* WifiPassword = "penbot123";

    // Planner & Shapes
    constexpr int CIRCLE_STEPS = 36;
    constexpr float SQUARE_GROWTH = 20.0f;

    // PID Parameters
    constexpr float PidLinearKp = 0.5f;
    constexpr float PidLinearKi = 0.05f;
    constexpr float PidLinearKd = 0.1f;
    constexpr float PidAngularKp = 0.3f;
    constexpr float PidAngularKi = 0.02f;
    constexpr float PidAngularKd = 0.05f;

    // Timing
    constexpr int LoopRateMsec = 10;
    constexpr int TelemetryRateMsec = 50;
    constexpr int StatusLedToggleMsec = 500;
}

// Aliases for main.cpp compatibility
using namespace Config;

#endif