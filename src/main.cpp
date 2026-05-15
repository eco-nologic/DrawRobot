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

// ============================================================================
// GLOBAL STATE
// ============================================================================
DCMotor leftMotor(PinMotorLeftEn, PinMotorLeftIn1, PinMotorLeftIn2, PinEncoderLeftA, PinEncoderLeftB, 0);
DCMotor rightMotor(PinMotorRightEn, PinMotorRightIn1, PinMotorRightIn2, PinEncoderRightA, PinEncoderRightB, 1);
DriveTrain drive(leftMotor, rightMotor);
Navigation nav;
PoseEstimator pose(leftMotor, rightMotor, nav);
MotionController motion(drive, pose);
PathPlanner planner(motion, nav);
CommsManager comms(planner);
BatteryMonitor battery;

unsigned long lastLoopTime = 0;
bool navReady = false;

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
    Serial.printf("DrawRobot Firmware v%s\n", Config::FirmwareVersion);
    Serial.println("========================================\n");

    Serial.println("[Boot] Mode: REAL HARDWARE");
}

void verifyMotorHardware() {
    Serial.println("[Boot] ⚙️ Verification de la chaine cinematique...");
    
    // DEFENSE: "Pourquoi testez-vous les moteurs au démarrage ?"
    // ANSWER: Pour garantir l'intégrité du matériel. Si un fil est débranché 
    // ou qu'un moteur est bloqué, on l'identifie avant de commencer le tracé.
    
    long startL = leftMotor.getTicks();
    long startR = rightMotor.getTicks();

    // Pulse de 250ms à 40% de puissance
    leftMotor.setSpeed(0.4f); 
    rightMotor.setSpeed(0.4f);
    delay(250);
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
    delay(100); // Stabilisation

    bool leftOk = abs(leftMotor.getTicks() - startL) > 5;
    bool rightOk = abs(rightMotor.getTicks() - startR) > 5;

    if (!leftOk || !rightOk) {
        if (!leftOk) Serial.println("[Boot] ❌ ERREUR: Echec moteur/encodeur GAUCHE !");
        if (!rightOk) Serial.println("[Boot] ❌ ERREUR: Echec moteur/encodeur DROIT !");
        digitalWrite(PinLedUser2, HIGH); // LED d'alerte
    } else {
        Serial.println("[Boot] ✅ Materiel moteur operationnel");
    }
}

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

    battery.begin();
    pose.begin();
    motion.begin();
    comms.begin();
    
    Serial.println("[Boot] Initialization complete!");
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
    if (now - lastTelemetry >= Config::TelemetryRateMsec) {
        Pose p = pose.getPenPose();
        comms.broadcastTelemetry(p.x, p.y, nav.getNavData().heading, battery.getVoltage());
        lastTelemetry = now;
    }

    delay(LoopRateMsec);
}