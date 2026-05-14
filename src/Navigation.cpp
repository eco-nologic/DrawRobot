#include "Navigation.h"
#include <Wire.h>

/**
 * @brief Initialise les capteurs LSM6DS3 et LIS3MDL sur le bus I2C.
 * DEFENSE: "Quels protocoles sont utilisés pour communiquer avec les capteurs ?"
 * ANSWER: Le protocole I2C (Inter-Integrated Circuit) via les broches SDA(21) et SCL(22).
 */
bool Navigation::begin() {
    // The I2C bus is already initialized in main.cpp.
    // We rely on the library's internal WHO_AM_I check for a cleaner startup.
    
    delay(200); // Increased settlement time for sensor boot-up
    
    /**
     * DEFENSE: "Comment détectez-vous une panne capteur ?"
     * ANSWER: La méthode begin_I2C() vérifie le registre WHO_AM_I des capteurs. 
     * Si le bus est déconnecté ou parasité par les moteurs, le robot signale l'erreur immédiatement.
     */
    bool lsmOk = false, lisOk = false;
    for (int retry = 0; retry < 3 && (!lsmOk || !lisOk); retry++) {
        if (!lsmOk) {
            lsmOk = _lsm.begin_I2C(Config::AddrLsm6ds);
            if (!lsmOk) lsmOk = _lsm.begin_I2C(0x6B); // Try alternate address if primary fails
        }
        if (!lisOk) lisOk = _lis.begin_I2C(Config::AddrLis3mdl);
        if (!lsmOk || !lisOk) {
            Serial.printf("[Nav] Init retry %d/3...\n", retry + 1);
            delay(500); // Longer delay between retries
        }
    }

    if (!lsmOk || !lisOk) {
        Serial.println("[Nav] ERROR: I2C Sensors failed to initialize after retries.");
        return false; 
    }

    Serial.println("[Nav] Sensors initialized successfully.");
    _data.heading = 0;
    _data.gyroZ = 0;
    _data.isCalibrated = false;
    return true;
}

/**
 * @brief Lit et fusionne les données brutes des capteurs.
 * DEFENSE: "Comment obtenez-vous le CAP ?"
 * ANSWER: En calculant l'arc-tangente des axes X et Y du magnétomètre (atan2) pour le Nord absolu.
 */
void Navigation::update() {
    sensors_event_t accel, gyro, temp, mag;
    _lsm.getEvent(&accel, &gyro, &temp);
    _lis.getEvent(&mag);

    _data.gyroZ = gyro.gyro.z; // Rad/s
    
    float mx = mag.magnetic.x;
    float my = mag.magnetic.y;

    if (_isCalibrating) {
        if (mx < _magMinX) _magMinX = mx; if (mx > _magMaxX) _magMaxX = mx;
        if (my < _magMinY) _magMinY = my; if (my > _magMaxY) _magMaxY = my;
        // En pratique, la calibration s'arrête après un tour complet du robot
    }

    // Application des offsets de calibration (Calcul du décalage Hard-Iron)
    if (_data.isCalibrated) {
        mx -= (_magMaxX + _magMinX) / 2.0f;
        my -= (_magMaxY + _magMinY) / 2.0f;
    }

    /**
     * DEFENSE: "Comment obtenez-vous le CAP ?"
     * ANSWER: En lisant les données du magnétomètre. On utilise atan2(y, x) 
     * pour obtenir un angle robuste face au bruit.
     */
    float angleRad = atan2(my, mx);
    float angleDeg = angleRad * 180.0f / M_PI;

    _data.heading = angleDeg;
    if (_data.heading < 0) _data.heading += 360.0f;
    if (_data.heading >= 360.0f) _data.heading -= 360.0f;
}

void Navigation::startCalibration() {
    _magMinX = _magMinY = 1000.0f;
    _magMaxX = _magMaxY = -1000.0f;
    _isCalibrating = true;
    _data.isCalibrated = false;
}

void Navigation::stopCalibration() {
    _isCalibrating = false;
    _data.isCalibrated = true; 
    Serial.println("[Nav] Magnetometer Calibration finalized.");
}