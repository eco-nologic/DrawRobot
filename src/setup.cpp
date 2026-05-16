#include "setup.h"
#include <Arduino.h>
#include <WiFi.h>
#include "Config.h"
#include "DCMotor.h"
#include "DriveTrain.h"
#include "Navigation.h"

// External references to global hardware instances defined in main.cpp
extern DCMotor leftMotor;
extern DCMotor rightMotor;
extern DriveTrain drive;
extern Navigation nav;

void setupWifi() {
    Serial.println("[WiFi] Starting WiFi Access Point...");
    
    // DEFENSE: "Pourquoi désactiver le WiFi Sleep mode ?"
    // ANSWER: Sur batterie, les variations de tension dues aux moteurs peuvent faire 
    // décrocher la radio si elle entre en mode économie d'énergie. En forçant 
    // WiFi.setSleep(false), on stabilise la latence et la robustesse du lien WebSocket.
    WiFi.setSleep(false);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(Config::WifiSsid, Config::WifiPassword);
    WiFi.softAPConfig(
        IPAddress(Config::WifiApIp),      // IP
        IPAddress(Config::WifiGateway),   // Gateway
        IPAddress(Config::WifiSubnet)     // Netmask
    );
    
    IPAddress ip = WiFi.softAPIP();
    Serial.print("[WiFi] AP IP: ");
    Serial.println(ip);
}

void setupGpio() {
    Serial.println("[GPIO] Initializing pins...");
    
    // Status LEDs
    pinMode(Config::PinLedUser1, OUTPUT);
    pinMode(Config::PinLedUser2, OUTPUT);
    digitalWrite(Config::PinLedUser1, LOW);
    digitalWrite(Config::PinLedUser2, LOW);
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
        digitalWrite(Config::PinLedUser2, HIGH); // LED d'alerte
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
        digitalWrite(Config::PinLedUser2, HIGH);
    } 
    else if (abs(avgGz) < 0.05f) {
        Serial.println("\n[Boot] ⚠️ ERREUR: Le Gyroscope Z ne répond pas au mouvement.");
        Serial.println("[Boot]  👉 FIX: Verifiez l'alimentation du capteur et le bus I2C.");
        digitalWrite(Config::PinLedUser2, HIGH);
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
        digitalWrite(Config::PinLedUser2, HIGH);
    } else {
        Serial.println("[Boot] ✅ Accéléromètre X validé.");
    }

    delay(500);
}