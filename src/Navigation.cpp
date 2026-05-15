#include "Navigation.h"
#include <Wire.h>

/**
 * @brief Initialise les capteurs LSM6DS3 et LIS3MDL sur le bus I2C.
 * DEFENSE: "Quels protocoles sont utilisés pour communiquer avec les capteurs ?"
 * ANSWER: Le protocole I2C (Inter-Integrated Circuit) via les broches SDA(21) et SCL(22).
 */
bool Navigation::begin() {
    delay(200); 
    
    if (SYSTEM_MODE == SystemRunMode::Real) {
        Serial.printf("\n[Nav] 🚀 Initialisation Navigation (Firmware %s)...\n", Config::FirmwareVersion);
    }
    
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

    // On lance la calibration du gyroscope au démarrage
    calibrate();

    _data.heading = 0;
    _data.gyroZ = 0;
    _data.isCalibrated = false;
    _lastUpdate = millis();
    return true;
}

void Navigation::calibrate() {
    // DEFENSE: "Pourquoi le robot doit-il rester immobile pendant le démarrage ?"
    // ANSWER: La fonction calibrate() calcule la moyenne de 200 échantillons du gyroscope. 
    // Si le robot bouge, le biais calculé sera faux, entraînant une dérive constante du cap.
    Serial.println("[Nav] ⚖️ Calibration Gyro... NE PAS BOUGER");
    float sumZ = 0, sumSqZ = 0;
    const int samples = 200;

    for (int i = 0; i < samples; i++) {
        sensors_event_t accel, gyro, temp;
        _lsm.getEvent(&accel, &gyro, &temp);
        float val = gyro.gyro.z;
        sumZ += val;
        sumSqZ += val * val; 
        if (i % 20 == 0) Serial.print(".");
        delay(5);
    }

    float mean = sumZ / samples;
    // MATH: Variance = E[X^2] - (E[X])^2 (Formule de Koenig-Huygens)
    // L'écart-type permet de vérifier statistiquement si le robot était bien immobile.
    float stdDev = sqrt(max(0.0f, (sumSqZ / samples) - (mean * mean)));

    if (stdDev > 0.02f) {
        Serial.printf("\n[Nav] ❌ ÉCHEC: Mouvement détecté (StdDev: %.4f)\n", stdDev);
        _gyroBiasZ = 0;
    } else {
        _gyroBiasZ = mean;
        Serial.printf("\n[Nav] ✅ Prêt. Biais: %.4f rad/s (StdDev: %.4f)\n", _gyroBiasZ, stdDev);
    }
}

/**
 * @brief Lit et fusionne les données brutes des capteurs.
 */
void Navigation::update() {
    unsigned long now = millis();
    float dt = (now - _lastUpdate) / 1000.0f;
    if (dt < 0.001f) return; 
    _lastUpdate = now;

    sensors_event_t accel, gyro, temp, mag;
    _lsm.getEvent(&accel, &gyro, &temp);
    _lis.getEvent(&mag);

    // 1. Tilt Compensation (Compensation d'inclinaison)
    // DEFENSE: "Comment gérez-vous l'inclinaison du robot ?"
    // ANSWER: On utilise l'accéléromètre pour calculer le Roll et le Pitch afin de projeter
    // le vecteur magnétique sur le plan horizontal (Xh, Yh).
    float roll = atan2(accel.acceleration.y, accel.acceleration.z);
    float pitch = atan2(-accel.acceleration.x, sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.z));

    float mx = mag.magnetic.x - _magOffsetX;
    float my = mag.magnetic.y - _magOffsetY;
    float mz = mag.magnetic.z;

    if (_isCalibrating) {
        if (mx < _magMinX) _magMinX = mx; if (mx > _magMaxX) _magMaxX = mx;
        if (my < _magMinY) _magMinY = my; if (my > _magMaxY) _magMaxY = my;
    }

    // MATH: Matrice de rotation inverse pour horizontaliser les mesures magnétiques.
    float Xh = mx * cos(pitch) + mz * sin(pitch);
    float Yh = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);

    float magHeading = atan2(-Yh, Xh);

    // 2. Filtre Complémentaire (Fusion Gyro + Mag)
    // DEFENSE: "Comment fusionnez-vous les données ?"
    // ANSWER: Le gyro donne la réactivité immédiate et le mag corrige la dérive lente.
    float gyroZRate = gyro.gyro.z - _gyroBiasZ;
    float error = magHeading - _headingIntegral;

    // MATH: Normalisation de l'erreur entre [-PI, PI]
    while (error > PI) error -= 2.0f * PI;
    while (error < -PI) error += 2.0f * PI;

    _headingIntegral += (gyroZRate + 0.05f * error) * dt;

    // Wrap final
    while (_headingIntegral > PI) _headingIntegral -= 2.0f * PI;
    while (_headingIntegral < -PI) _headingIntegral += 2.0f * PI;

    _data.gyroZ = gyroZRate;
    _data.heading = _headingIntegral * 180.0f / PI; 
    if (_data.heading < 0) _data.heading += 360.0f;
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