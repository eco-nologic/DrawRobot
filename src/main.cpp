#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include "Config.h"
#include "DCMotor.h"
#include "DriveTrain.h"
#include "Navigation.h"
#include "PoseEstimator.h"
#include "MotionController.h"
#include "PathPlanner.h"
#include "CommsManager.h"
#include "BatteryMonitor.h"
#include <LittleFS.h>
#include "BluetoothManager.h" // Inclure BluetoothManager
#include "TelemetryPacket.h"  // Inclure TelemetryPacket
#include "setup.h"
#include "CommandHandler.h"

// ============================================================================
// GLOBAL STATE
// ============================================================================
DCMotor leftMotor(PinMotorLeftEn, PinMotorLeftIn1, PinMotorLeftIn2, PinEncoderLeftA, PinEncoderLeftB, 0);
DCMotor rightMotor(PinMotorRightEn, PinMotorRightIn1, PinMotorRightIn2, PinEncoderRightA, PinEncoderRightB, 1);
DriveTrain drive(leftMotor, rightMotor);
Navigation nav;
PoseEstimator pose(leftMotor, rightMotor, nav);
MotionController motion(drive, pose);
PathPlanner planner(motion, nav); // PathPlanner a besoin de MotionController et Navigation
CommandHandler cmdHandler(planner, drive);
CommsManager comms(cmdHandler);
BatteryMonitor battery;
BluetoothManager ble(cmdHandler); // Instance globale du BluetoothManager

unsigned long lastLoopTime = 0;
bool navReady = false;

// ============================================================================
// INITIALIZATION
// ============================================================================

void setup() {
    setupSerial();
    setupGpio();
    setupWifi();

    // --- FILESYSTEM ---
    if (!LittleFS.begin(true)) {
        Serial.println("[Boot] ❌ LittleFS Mount Failed!");
    } else {
        Serial.println("[Boot] ✅ LittleFS Mounted");
    }

    // Initialisation et test des moteurs AVANT la navigation/calibration
    if (Config::SYSTEM_MODE == Config::SystemRunMode::Real) {
        drive.begin();
        verifyMotorHardware();
    }

    // --- I2C BUS RECOVERY ---
    // Perform bit-banging recovery BEFORE initializing the Wire peripheral
    Serial.println("[I2C] Performing bus recovery...");
    pinMode(Config::SDA, OUTPUT);
    pinMode(Config::SCL, OUTPUT);
    
    // Generate 10 clock pulses to release any slave holding SDA low
    for (int i = 0; i < 10; i++) {
        digitalWrite(Config::SCL, LOW);
        delayMicroseconds(10);
        digitalWrite(Config::SCL, HIGH);
        delayMicroseconds(10);
    }
    
    // Set pins to INPUT_PULLUP to ensure high state before Wire takeover
    pinMode(Config::SDA, INPUT_PULLUP);
    pinMode(Config::SCL, INPUT_PULLUP);
    delay(50);

    // Initialize I2C bus once with config parameters
    Wire.begin(Config::SDA, Config::SCL, Config::I2C_FREQ);
    delay(100); // Wait for the peripheral and bus to stabilize
    
    if (!(navReady = nav.begin())) {
        Serial.println("[Boot] ❌ Navigation initialization failed! Check I2C wiring.");
        // Optional: Blink an error LED
        digitalWrite(PinLedUser2, HIGH); 
    }

    // Test de l'IMU en mouvement (si nav est prête)
    if (navReady && Config::SYSTEM_MODE == Config::SystemRunMode::Real) {
        verifyNavigationLogic();
    }

    battery.begin();
    pose.begin();
    motion.begin();
    comms.begin();
    
    Serial.println("[Boot] Initialization complete!");

    // Initialisation du BluetoothManager
    if (!ble.begin()) {
        Serial.println("[WARN] BluetoothManager initialization failed!");
    } else {
        Serial.println("[Boot] ✅ BluetoothManager initialized");
    }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    unsigned long now = millis();
    float dt = (now - lastLoopTime) / 1000.0f;
    lastLoopTime = now;

    // 1. Update Sensors & Localization (Couche Basse)
    if (navReady) nav.update();
    battery.update();
    pose.update();

    // 2. Update Control Logic (Couche Moyenne - PID)
    // On met à jour le contrôleur de mouvement indépendamment du planificateur
    motion.update(dt);

    // 3. Update Sequence Planner (Couche Haute - Stratégie)
    planner.update(dt);

    // 4. Telemetry broadcast
    static unsigned long lastTelemetry = 0;

    // DEFENSE: "Pourquoi la télémétrie est-elle ralentie en mode Segway ?"
    // ANSWER: Pour garantir la priorité maximale à la boucle d'équilibre (100Hz). 
    // En mode normal, on envoie à 10Hz. En mode Segway, on réduit à 1Hz pour éviter que 
    // les interruptions WiFi/BLE ne créent des micro-retards fatals à la stabilité.
    unsigned long telemetryInterval = planner.isBalancing() ? 1000 : Config::TelemetryRateMsec;

    if (now - lastTelemetry >= telemetryInterval) {
        Pose p = pose.getPenPose();
        
        // Construction du paquet de télémétrie complet
        TelemetryPacket telem;
        
        // Pose du robot (stylo)
        telem.robotX = p.x;
        telem.robotY = p.y;
        telem.robotHeading = nav.getNavData().heading; // Cap fusionné en degrés

        // DEFENSE: "À quoi servent les données 'Ghost' ?"
        // ANSWER: À quantifier l'apport de l'IMU. On compare la position estimée par les roues 
        // seules (Ghost) à la position fusionnée pour détecter les dérives odométriques.        
        Pose ghost = pose.getGhostPose();
        telem.ghostX = ghost.x;
        telem.ghostY = ghost.y;
        telem.ghostHeading = ghost.theta * (180.0f / M_PI);

        // Moteurs
        telem.leftWheelSpeed = 0; // Non implémenté dans DriveTrain.cpp
        telem.rightWheelSpeed = 0;
        telem.leftWheelSteps = leftMotor.getTicks();
        telem.rightWheelSteps = rightMotor.getTicks();

        // IMU (données brutes)
        ImuData rawImu = nav.getRawData();
        telem.accelX = rawImu.accelX; telem.accelY = rawImu.accelY; telem.accelZ = rawImu.accelZ;
        telem.gyroX = rawImu.gyroX; telem.gyroY = rawImu.gyroY; telem.gyroZ = rawImu.gyroZ;
        telem.magX = rawImu.magX; telem.magY = rawImu.magY; telem.magZ = rawImu.magZ;

        // Autres
        telem.batteryVoltage = battery.getVoltage();
        telem.isMoving = !motion.isReached(); 
        telem.isCalibrated = nav.isCalibrated();
        telem.waypointIndex = 0; 
        // Les cibles X, Y et le bearingToTarget peuvent être ajoutés si MotionController les expose
        telem.targetX = 0; telem.targetY = 0; telem.bearingToTarget = 0;

        telem.targetL = planner.getTargetL();
        telem.targetTheta = planner.getTargetTheta();
        telem.targetR = planner.getTargetR();

        // Envoi de la télémétrie via WebSocket (CommsManager)
        comms.broadcastTelemetry(telem);
        // Envoi de la télémétrie via BLE (BluetoothManager)
        ble.sendTelemetry(telem);
        lastTelemetry = now;
    }

    delay(LoopRateMsec);
}