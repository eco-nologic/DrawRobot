#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <cmath>

namespace Config {
    // Mode de fonctionnement du système
    enum class SystemRunMode { Real, Sim };
    #ifdef MODE_SIM
      constexpr SystemRunMode SYSTEM_MODE = SystemRunMode::Sim;
    #else
      constexpr SystemRunMode SYSTEM_MODE = SystemRunMode::Real;
    #endif

    // Firmware
    constexpr char FirmwareVersion[] = "1.2.22";

    // Physical Parameters
    // DEFENSE: "Comment déterminez-vous la distance parcourue par tick ?"
    // ANSWER: On divise la circonférence de la roue (D * PI) par la résolution de l'encodeur (1070).
    constexpr float WHEEL_DIAMETER = 90.0f;   // Diamètre de la roue en mm
    
    // DEFENSE: "À quoi sert le WHEEL_BASE dans vos calculs ?"
    // ANSWER: C'est l'entraxe (L). Il est indispensable pour convertir une vitesse angulaire 
    // souhaitée en différence de vitesse entre la roue gauche et la roue droite.
    constexpr float WHEEL_BASE = 83.0f;       
    
    // DEFENSE: "Pourquoi le stylo n'est-il pas au centre de l'essieu ?"
    // ANSWER: La conception mécanique place le stylo à 130mm. Mathématiquement, on doit 
    // compenser ce décalage pour que le tracé corresponde à la trajectoire (cinématique du bras de levier).
    constexpr float PEN_OFFSET = 130.0f;      
    
    constexpr int STEPS_PER_REV = 1070;       // Résolution totale de l'encodeur
    
    // MATH: mm_par_step = (Circonférence) / (Ticks_par_tour)
    constexpr float MM_PER_STEP = (WHEEL_DIAMETER * M_PI) / STEPS_PER_REV;
    constexpr float MM_PER_TICK = MM_PER_STEP; // Utilisé par le PoseEstimator pour l'intégration de la distance

    // Motor Pins
    constexpr int PinMotorLeftEn = 4;
    constexpr int PinMotorLeftIn1 = 17; 
    constexpr int PinMotorLeftIn2 = 16; 
    constexpr int PinMotorRightEn = 23;
    constexpr int PinMotorRightIn1 = 18; // Swapped to correct forward direction
    constexpr int PinMotorRightIn2 = 19; // Swapped to correct forward direction

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
    // DEFENSE: "Comment le robot corrige-t-il sa trajectoire en temps réel ?"
    // ANSWER: Via deux boucles PID. Le PID linéaire gère la distance vers la cible 
    // et le PID angulaire maintient l'orientation vers le waypoint.
    constexpr float PidLinearKp = 0.5f;   // Réaction à l'erreur de distance
    constexpr float PidLinearKi = 0.05f;  // Compensation des frottements (erreur statique)
    constexpr float PidLinearKd = 0.1f;
    constexpr float PidAngularKp = 0.3f;  // Réaction aux écarts de cap
    constexpr float PidAngularKi = 0.02f;
    constexpr float PidAngularKd = 0.05f;

    // Timing
    constexpr int LoopRateMsec = 10;
    // DEFENSE: "Pourquoi avoir réduit la fréquence de la télémétrie ?"
    // ANSWER: Pour prioriser les ressources CPU sur la boucle de navigation et la correction 
    // de trajectoire (100Hz). Réduire la télémétrie à 10Hz (100ms) diminue la charge 
    // du bus WiFi/WebSocket sans impacter la perception humaine du mouvement sur le dashboard.
    constexpr int TelemetryRateMsec = 100;
    constexpr int StatusLedToggleMsec = 500;
}

// Aliases for main.cpp compatibility
using namespace Config;

#endif