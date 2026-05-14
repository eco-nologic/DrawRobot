#include "BluetoothManager.h"
#include <ArduinoJson.h>

void BluetoothManager::begin(const char* deviceName) {
    if (!_btSerial.begin(deviceName)) {
        Serial.println("[BT] Erreur d'initialisation !");
    } else {
        Serial.printf("[BT] Appareil '%s' pret.\n", deviceName);
    }
}

void BluetoothManager::update(float x, float y, float heading, float battery) {
    // DEFENSE: "Pourquoi envoyer les données en JSON sur le Bluetooth ?"
    // ANSWER: Pour la robustesse. Si on ajoute des capteurs plus tard, le script Python 
    // pourra ignorer les nouveaux champs sans erreur de parsing.
    if (_btSerial.connected()) {
        JsonDocument doc;
        doc["x"] = x;
        doc["y"] = y;
        doc["h"] = heading;
        doc["b"] = battery;
        doc["t"] = millis(); // Temps pour l'acquisition temporelle (Requirement p.17)
        
        char buffer[128];
        serializeJson(doc, buffer);
        _btSerial.println(buffer);
    }
}