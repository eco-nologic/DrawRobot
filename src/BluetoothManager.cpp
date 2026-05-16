#include "BluetoothManager.h"

// Callbacks pour gérer la connexion/déconnexion
class MyServerCallbacks : public BLEServerCallbacks {
    BluetoothManager* _manager;
public:
    MyServerCallbacks(BluetoothManager* manager) : _manager(manager) {}
    void onConnect(BLEServer* pServer) {
        _manager->isConnected = true;
        Serial.println("[BLE] 📲 Client connecté");
    }
    void onDisconnect(BLEServer* pServer) {
        _manager->isConnected = false;
        Serial.println("[BLE] 📴 Client déconnecté");
        // Relancer la publicité pour permettre une nouvelle connexion
        BLEDevice::startAdvertising();
    }
};

// Callbacks pour gérer la réception de commandes (WRITE)
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
    BluetoothManager* _manager;
public:
    MyCharacteristicCallbacks(BluetoothManager* manager) : _manager(manager) {}
    void onWrite(BLECharacteristic* pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        _manager->processIncomingCommand((uint8_t*)rxValue.c_str(), rxValue.length());
    }
};

BluetoothManager::BluetoothManager(CommandHandler& handler) 
    : isConnected(false), _isInitialized(false), _commandHandler(handler) {}

bool BluetoothManager::begin() {
    Serial.println("[BLE] Initialisation du service BLE...");
    
    try {
        BLEDevice::init("GiRobot_BLE");
        pServer = BLEDevice::createServer();
        if (!pServer) return false;

        pServer->setCallbacks(new MyServerCallbacks(this));

        BLEService* pService = pServer->createService(SERVICE_UUID);
        if (!pService) return false;

        pCharacteristic = pService->createCharacteristic(
            CHARACTERISTIC_UUID,
            BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE
        );
        
        if (!pCharacteristic) return false;

        pCharacteristic->setCallbacks(new MyCharacteristicCallbacks(this));

        pCharacteristic->addDescriptor(new BLE2902());
        pService->start();

        BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
        pAdvertising->addServiceUUID(SERVICE_UUID);
        pAdvertising->setScanResponse(true);
        BLEDevice::startAdvertising();

        Serial.println("[BLE] ✅ Serveur actif. Nom: 'GiRobot_BLE'");
        _isInitialized = true;
        return true;
    } catch (...) {
        _isInitialized = false;
        return false;
    }
}

void BluetoothManager::sendTelemetry(const TelemetryPacket& packet) {
    // DEFENSE: "Pourquoi vérifier la connexion avant d'envoyer ?"
    // ANSWER: L'envoi de notifications BLE consomme de l'énergie et du temps CPU. 
    // Si aucun client n'est connecté, on économise la batterie en ne formatant pas la chaîne.
    if (!isConnected || !_isInitialized) return;

    char buffer[256];
    snprintf(buffer, sizeof(buffer), 
             "A:%.2f,%.2f,%.2f|G:%.2f,%.2f,%.2f|M:%.2f,%.2f,%.2f|H:%.2f|B:%.2f|T:%lu|X:%.1f|Y:%.1f|Lth:%.1f|Ath:%.1f|Rth:%.1f",
             packet.accelX, packet.accelY, packet.accelZ,
             packet.gyroX, packet.gyroY, packet.gyroZ,
             packet.magX, packet.magY, packet.magZ,
             packet.robotHeading,
             packet.batteryVoltage,
             millis(),
             packet.robotX, packet.robotY,
             packet.targetL, packet.targetTheta, packet.targetR);

    if (pCharacteristic != nullptr) {
        pCharacteristic->setValue(buffer);
        pCharacteristic->notify();
        lastPacketTime = millis();
    }
}

void BluetoothManager::processIncomingCommand(const uint8_t* data, size_t length) {
    // DEFENSE: "Comment le BLE communique avec le reste du robot ?"
    // ANSWER: Les octets reçus sont transmis au CommandHandler central. 
    // Ce dernier désérialise le JSON et exécute la logique de mouvement.
    // On utilise const_cast car CommandHandler attend un pointeur modifiable pour ArduinoJson
    _commandHandler.processJSONCommand(const_cast<uint8_t*>(data), length);
}

void BluetoothManager::stop() {
    if (_isInitialized) {
        BLEDevice::deinit(false);
        _isInitialized = false;
        isConnected = false;
    }
}

bool BluetoothManager::isActivated() const {
    return _isInitialized;
}