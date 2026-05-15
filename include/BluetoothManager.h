#ifndef BLUETOOTH_MANAGER_H
#define BLUETOOTH_MANAGER_H

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "Config.h"
#include "TelemetryPacket.h" // Inclusion de la structure TelemetryPacket

/**
 * @class BluetoothManager
 * @brief Gère la télémétrie "Out-of-band" via Bluetooth Serial pour le rapport de validation.
 */
class BluetoothManager {
public:
    BluetoothManager();

    // DEFENSE: "Pourquoi utiliser le Bluetooth en plus du WiFi ?"
    // ANSWER: Pour disposer d'un canal de données indépendant. Si le WiFi sature lors de l'envoi 
    // de commandes lourdes, le Bluetooth permet de récupérer les logs de précision sans interférence.
    bool begin();

    // DEFENSE: "Comment le Bluetooth aide-t-il à la validation (p.17 du rapport) ?"
    // ANSWER: Il permet de streamer les coordonnées (x,y,t) vers un script Python sur PC 
    // qui enregistre un fichier CSV. Ce fichier est ensuite utilisé pour tracer les graphiques lmes vs lth.
    void sendTelemetry(const TelemetryPacket& packet);

    // Pour la gestion des commandes entrantes via BLE (non implémenté ici)
    void processIncomingCommand(const uint8_t* data, size_t length);

    // Arrête le service BLE
    void stop();

    // Vérifie si le module BLE est initialisé et actif
    bool isActivated() const;

    // Indique si un client BLE est connecté
    bool isConnected;
    void setConnected(bool state) { isConnected = state; }

private:
    BLEServer* pServer = nullptr;
    BLECharacteristic* pCharacteristic = nullptr;
    bool _isInitialized = false; // Flag pour vérifier si BLE a été activé avec succès
    unsigned long lastPacketTime;

    // UUIDs pour le service et la caractéristique (doivent correspondre au script Python)
    const char* SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
    const char* CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
};

#endif