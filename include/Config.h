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
    constexpr char FirmwareVersion[] = "1.2.70";

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
    constexpr float MaxLinearSpeedMmS = 120.0f; // Vitesse linéaire maximale (mm/s)
    
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
    constexpr uint8_t WifiApIp[4] = {192, 168, 4, 1};
    constexpr uint8_t WifiGateway[4] = {192, 168, 4, 1};
    constexpr uint8_t WifiSubnet[4] = {255, 255, 255, 0};

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

    // --- EASTER EGG: Balancing Mode (Segway Style) ---
    // DEFENSE: "Comment le robot tient-il debout sur deux roues ?"
    // ANSWER: Via un PID à haute fréquence (100Hz) qui compense l'erreur de tangage (Pitch).
    // DEFENSE: "Pourquoi les moteurs tournaient-ils à fond à la main ?"
    // ANSWER: Le gain Kp était trop élevé (150). Une inclinaison de 1° provoquait une saturation 
    // immédiate des moteurs. Nous avons réduit Kp à 45 pour une réponse plus proportionnelle 
    // et augmenté Kd pour amortir l'effet ressort.
    constexpr float PidBalanceKp = 45.0f; 
    constexpr float PidBalanceKi = 2.0f; 
    constexpr float PidBalanceKd = 8.0f; 
    
    // MATH: Le point d'équilibre est quand l'axe X du robot est parfaitement vertical (gravité).
    // atan2(accelZ, accelX) devrait être 0 quand le robot est debout (accelZ=0, accelX=9.81).
    constexpr float BalancePointRad = 0.0f; // Angle cible pour le Pitch (0 = vertical)

    // --- Kalman Filter Parameters for Balance (Easter Egg) ---
    // DEFENSE: "Comment le filtre de Kalman rend-il l'équilibre plus stable ?"
    // ANSWER: Il modélise l'incertitude du capteur et du processus.
    // Q_angle: Bruit du processus (intégration du gyro), Q_bias: Bruit du biais du gyro.
    // R_measure: Bruit de la mesure (accéléromètre).
    constexpr float KalmanQAngle = 0.001f; // Bruit du processus sur l'angle
    constexpr float KalmanQBias = 0.003f;  // Bruit du processus sur le biais du gyroscope
    constexpr float KalmanRMeasure = 0.03f; // Bruit de la mesure (accéléromètre)
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