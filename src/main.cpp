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
CommsManager comms(planner);
BatteryMonitor battery;
BluetoothManager ble; // Instance globale du BluetoothManager

unsigned long lastLoopTime = 0;
bool navReady = false;

// ============================================================================
// INITIALIZATION
// ============================================================================

void setupWifi() {
    Serial.println("[WiFi] Starting WiFi Access Point...");
    
    // DEFENSE: "Pourquoi désactiver le WiFi Sleep mode ?"
    // ANSWER: Sur batterie, les variations de tension dues aux moteurs peuvent faire 
    // décrocher la radio si elle entre en mode économie d'énergie. En forçant 
    // WiFi.setSleep(false), on stabilise la latence et la robustesse du lien WebSocket.
    WiFi.setSleep(false);

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

void verifyNavigationLogic() {
    // DEFENSE: "Comment validez-vous la cohérence entre moteurs et capteurs ?"
    // ANSWER: Par un test dynamique au boot. On commande un virage à droite et on vérifie 
    // que le gyroscope Z renvoie une valeur négative (sens horaire). 
    // Cela garantit que la boucle fermée du PID ne divergera pas.

    Serial.println("[Boot] 🧪 Verification dynamique de l'IMU...");
    
    // --- TEST 1: Virage à Droite (Check Gyro Z) ---
    // En robotique, un virage à droite (Clockwise) doit produire une vitesse angulaire négative.
    Serial.println("[Boot]  -> Test Rotation Droite...");
    drive.setVelocity(0.0f, -1.5f); // Vitesse angulaire négative (Droite)
    
    float sumGz = 0;
    for(int i=0; i<10; i++) {
        nav.update();
        sumGz += nav.getNavData().gyroZ;
        delay(50);
    }
    drive.stop();

    float avgGz = sumGz / 10.0f;
    
    if (avgGz > 0.1f) {
        Serial.println("\n[Boot] ❌ ERREUR: Direction du Gyroscope Z inversée !");
        Serial.printf("[Boot]  Mesuré: %.2f rad/s (Attendu: < 0)\n", avgGz);
        Serial.println("[Boot]  👉 FIX: Dans Navigation.cpp, inversez le signe de rawData.gyroZ");
        digitalWrite(PinLedUser2, HIGH);
    } 
    else if (abs(avgGz) < 0.05f) {
        Serial.println("\n[Boot] ⚠️ ERREUR: Le Gyroscope Z ne répond pas au mouvement.");
        Serial.println("[Boot]  👉 FIX: Verifiez l'alimentation du capteur et le bus I2C.");
        digitalWrite(PinLedUser2, HIGH);
    }
    else {
        Serial.println("[Boot] ✅ Gyroscope Z en phase avec les moteurs.");
    }

    delay(200);

    // --- TEST 2: Accélération Linéaire (Check Accel X) ---
    // DEFENSE: "Comment vérifiez-vous l'accéléromètre ?"
    // ANSWER: On envoie une impulsion brusque vers l'avant. L'accéléromètre doit détecter 
    // une variation sur l'axe X (longitudinal). Cela valide que le remapping sensor.Y -> robot.X est correct.
    
    Serial.println("[Boot]  -> Test Accélération Avant...");
    float baselineAx = nav.getRawData().accelX;
    drive.setVelocity(0.8f, 0.0f); // Pulse de vitesse linéaire
    
    float maxDev = 0;
    for(int i=0; i<8; i++) {
        nav.update();
        float dev = abs(nav.getRawData().accelX - baselineAx);
        if (dev > maxDev) maxDev = dev;
        delay(30);
    }
    drive.stop();

    if (maxDev < 0.25f) {
        Serial.println("\n[Boot] ❌ ERREUR: L'accéléromètre X ne détecte aucune poussée !");
        Serial.printf("[Boot]  Variation max: %.2f m/s² (Attendu: > 0.25)\n", maxDev);
        Serial.println("[Boot]  👉 FIX: Vérifiez si l'axe longitudinal est bien mappé sur rawData.accelX.");
        digitalWrite(PinLedUser2, HIGH);
    } else {
        Serial.println("[Boot] ✅ Accéléromètre X validé.");
    }

    // Petite pause pour stabiliser le robot avant la suite
    delay(500);
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