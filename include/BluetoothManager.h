#ifndef BLUETOOTH_MANAGER_H
#define BLUETOOTH_MANAGER_H

#include <BluetoothSerial.h>
#include "Config.h"

/**
 * @class BluetoothManager
 * @brief Gère la télémétrie "Out-of-band" via Bluetooth Serial pour le rapport de validation.
 */
class BluetoothManager {
public:
    BluetoothManager() = default;

    // DEFENSE: "Pourquoi utiliser le Bluetooth en plus du WiFi ?"
    // ANSWER: Pour disposer d'un canal de données indépendant. Si le WiFi sature lors de l'envoi 
    // de commandes lourdes, le Bluetooth permet de récupérer les logs de précision sans interférence.
    void begin(const char* deviceName = "DrawRobot-ECE");

    // DEFENSE: "Comment le Bluetooth aide-t-il à la validation (p.17 du rapport) ?"
    // ANSWER: Il permet de streamer les coordonnées (x,y,t) vers un script Python sur PC 
    // qui enregistre un fichier CSV. Ce fichier est ensuite utilisé pour tracer les graphiques lmes vs lth.
    void update(float x, float y, float heading, float battery);

private:
    BluetoothSerial _btSerial;
};

#endif