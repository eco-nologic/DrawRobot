#include "Navigation.h"
#include <Wire.h>
#include <math.h> // Pour M_PI, cos, sin, atan2, sqrt

/**
 * @brief Initialise les capteurs LSM6DS3 et LIS3MDL sur le bus I2C.
 * DEFENSE: "Quels protocoles sont utilisés pour communiquer avec les capteurs ?"
 * ANSWER: Le protocole I2C (Inter-Integrated Circuit) via les broches SDA(21) et SCL(22).
 */
Navigation::Navigation() {
    memset(&rawData, 0, sizeof(ImuData));
    memset(&orientation, 0, sizeof(OrientationData));
}

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
    // DEFENSE: "Comment gérez-vous les variations d'adresses I2C du matériel ?"
    // ANSWER: Nous utilisons une logique de scan sur les deux adresses standards (0x6A/0x6B et 0x1C/0x1E).
    // Cela permet au firmware d'être compatible avec différentes révisions de cartes électroniques.

    // 1. Détection du LSM6DS3 (Accéléromètre / Gyroscope)
    bool lsmFound = false;
    uint8_t lsmAddrs[] = {Config::AddrLsm6ds, 0x6B}; 
    for (uint8_t addr : lsmAddrs) {
        if (_lsm.begin_I2C(addr, &Wire)) {
            Serial.printf("[Nav] ✅ LSM6DS3 trouvé à l'adresse 0x%02X\n", addr);
            lsmFound = true;
            break;
        }
        delay(10);
    }

    if (!lsmFound) {
        Serial.println("[Nav] ❌ Erreur critique : LSM6DS3 introuvable sur le bus I2C.");
        return false;
    }
    
    delay(100); // Temps de stabilisation accru

    // 2. Détection du LIS3MDL (Magnétomètre)
    bool lisFound = false;
    uint8_t lisAddrs[] = {Config::AddrLis3mdl, 0x1C};
    for (uint8_t addr : lisAddrs) {
        if (_lis.begin_I2C(addr, &Wire)) {
            Serial.printf("[Nav] ✅ LIS3MDL trouvé à l'adresse 0x%02X\n", addr);
            lisFound = true;
            break;
        }
        delay(10);
    }

    if (!lisFound) {
        Serial.println("[Nav] ❌ Erreur critique : LIS3MDL introuvable sur le bus I2C.");
        return false;
    }

    Serial.println("[Nav] Sensors initialized successfully.");
    Serial.println("[Nav] ✅ LSM6DS3 & LIS3MDL hardware initialized");

    // On lance la calibration du gyroscope au démarrage
    calibrate();

    _data.heading = 0;
    _data.gyroZ = 0;
    _data.isCalibrated = false;
    _lastUpdate = millis();
    _headingIntegral = 0; // Initialisation de l'intégrale du cap
    return true;
}

void Navigation::calibrate() {
    // DEFENSE: "Pourquoi le robot doit-il rester immobile pendant le démarrage ?"
    // ANSWER: La fonction calibrate() calcule la moyenne de 200 échantillons du gyroscope. 
    // Nous bouclons maintenant jusqu'à obtenir un écart-type < 0.015, garantissant une stabilité parfaite.
    
    bool isStable = false;
    while (!isStable) {
        Serial.println("[Nav] ⚖️ Calibration Gyroscope... NE PAS BOUGER");
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
        /**
         * MATH: Formule de Koenig-Huygens pour la variance : Var(X) = E[X^2] - (E[X])^2.
         * Cette méthode permet de calculer la dispersion (bruit) sans stocker les 200 points en mémoire.
         */
        float variance = (sumSqZ / samples) - (mean * mean);
        float stdDev = sqrt(fmax(0.0f, variance));

        if (stdDev < 0.015f) { // Seuil de stabilité strict
            _gyroBiasZ = mean;
            Serial.printf("\n[Nav] ✅ Stabilité atteinte. Biais: %.4f rad/s (StdDev: %.4f)\n", _gyroBiasZ, stdDev);
            isStable = true;
        } else {
            Serial.printf("\n[Nav] ⚠️ Instable (StdDev: %.4f). Nouvel essai dans 1s...\n", stdDev);
            delay(1000);
        }
    }

    // MATH: On force l'initialisation de l'intégrale à 0 pour que update() déclenche 
    // l'alignement immédiat sur le magnétomètre au premier cycle.
    _headingIntegral = 0; 
    _lastUpdate = 0;
}

/**
 * @brief Lit et fusionne les données brutes des capteurs.
 * DEFENSE: "Comment le robot obtient-il son orientation ?"
 * ANSWER: En lisant les données brutes de l'accéléromètre, du gyroscope et du magnétomètre,
 * puis en les fusionnant via un filtre complémentaire pour obtenir un cap stable et réactif.
 * Les données brutes sont stockées dans `rawData` et l'orientation fusionnée dans `orientation`.
 */
void Navigation::update() {
    unsigned long now = millis();
    float dt = (now - _lastUpdate) / 1000.0f;
    if (dt < 0.001f) return; 
    _lastUpdate = now;

    // Lecture des données brutes des capteurs
    sensors_event_t accelEvent, gyroEvent, tempEvent, magEvent;
    _lsm.getEvent(&accelEvent, &gyroEvent, &tempEvent);
    _lis.getEvent(&magEvent);

    /**
     * DEFENSE: "Pourquoi remappez-vous les axes des capteurs ?"
     * ANSWER: Selon le plan mécanique (travail.txt), l'axe longitudinal (avant) du robot est Y.
     * Nous convertissons les données du capteur (qui a sa propre orientation) vers le repère du robot
     * (X=Avant, Y=Gauche, Z=Haut) pour que les algorithmes de navigation standard fonctionnent.
     */
    // Remap axes: Sensor Y -> Robot X (Forward), Sensor -X -> Robot Y (Left), Sensor Z -> Robot Z (Up)
    rawData.accelX = accelEvent.acceleration.y;
    rawData.accelY = -accelEvent.acceleration.x;
    rawData.accelZ = accelEvent.acceleration.z;
    
    rawData.gyroX = gyroEvent.gyro.y;
    rawData.gyroY = -gyroEvent.gyro.x;

    // DEFENSE: "Pourquoi l'axe Z du gyroscope est-il inversé ?"
    // ANSWER: La convention robotique impose une vitesse positive pour un virage à gauche. 
    // Le test verifyNavigationLogic() a détecté que le capteur physique renvoyait des valeurs 
    // positives lors d'un virage à droite. L'inversion ici garantit la stabilité du PID.
    rawData.gyroZ = -gyroEvent.gyro.z; 

    rawData.magX = magEvent.magnetic.y;
    rawData.magY = -magEvent.magnetic.x;
    rawData.magZ = magEvent.magnetic.z;

    // Appliquer les offsets Hard-Iron (calculés via analyze_imu.py)
    float correctedMagX = rawData.magX - _magOffsetX;
    float correctedMagY = rawData.magY - _magOffsetY;
    float correctedMagZ = rawData.magZ - _magOffsetZ; // Si un offset Z est nécessaire

    // La calibration du magnétomètre (Hard-Iron) se fait en arrière-plan
    // pendant la séquence de calibration (rotation du robot).
    // Les valeurs min/max sont utilisées pour calculer les offsets.
    if (_isCalibrating) {
        if (correctedMagX < _magMinX) _magMinX = correctedMagX; if (correctedMagX > _magMaxX) _magMaxX = correctedMagX;
        if (correctedMagY < _magMinY) _magMinY = correctedMagY; if (correctedMagY > _magMaxY) _magMaxY = correctedMagY;
    }

    // 1. Calcul de l'assiette (Roll/Pitch) pour la compensation d'inclinaison
    // DEFENSE: "Comment gérez-vous l'inclinaison du robot pour le cap ?"
    // ANSWER: On utilise l'accéléromètre pour calculer le Roll et le Pitch.
    // Ces angles sont ensuite utilisés pour projeter le vecteur magnétique sur le plan horizontal.
    // MATH: Roll = atan2(Ay, Az), Pitch = atan2(-Ax, sqrt(Ay^2 + Az^2))
    orientation.roll = atan2(rawData.accelY, rawData.accelZ);
    orientation.pitch = atan2(-rawData.accelX, sqrt(rawData.accelY * rawData.accelY + rawData.accelZ * rawData.accelZ));

    /**
     * MATH: Compensation d'inclinaison (Tilt Compensation).
     * On applique une matrice de rotation inverse basée sur Pitch (P) et Roll (R) pour projeter
     * le vecteur magnétique sur un plan horizontal virtuel (Z=0).
     * Xh = mx*cos(P) + mz*sin(P)
     * Yh = mx*sin(R)*sin(P) + my*cos(R) - mz*sin(R)*cos(P)
     */
    float Xh = correctedMagX * cos(orientation.pitch) + correctedMagZ * sin(orientation.pitch);
    float Yh = correctedMagX * sin(orientation.roll) * sin(orientation.pitch) + correctedMagY * cos(orientation.roll) - correctedMagZ * sin(orientation.roll) * cos(orientation.pitch);

    // Calcul du cap magnétique absolu
    /**
     * MATH: atan2(y, x) convertit les coordonnées cartésiennes en angle polaire (radians).
     * On utilise -Yh car, en robotique, le cap (Heading) augmente dans le sens horaire,
     * à l'inverse du sens trigonométrique standard.
     */
    float magHeading = atan2(-Yh, Xh);

    // 2. Filtre Complémentaire (Fusion Gyro + Mag)
    /**
     * MATH: Filtre Complémentaire (Fusion Sensorielle).
     * On combine la réactivité du gyroscope (haute fréquence) et la stabilité du magnétomètre (basse fréquence).
     * Équation : θ_n = θ_{n-1} + (ω_gyro + Kp * (θ_mag - θ_{n-1})) * dt
     * Le gain Kp (0.05) définit la vitesse de correction de la dérive (drift).
     */
    float gyroZRate = rawData.gyroZ - _gyroBiasZ; // Soustraire le biais du gyroscope
    float headingError = magHeading - _headingIntegral;

    // MATH: Normalisation de l'erreur entre [-PI, PI] pour éviter les problèmes
    // de discontinuité (ex: passer de 179° à -179°).
    while (headingError > M_PI) headingError -= 2.0f * M_PI;
    while (headingError < -M_PI) headingError += 2.0f * M_PI;

    // MATH: Intégration temporelle du gyroscope avec correction du magnétomètre.
    // Le gain 0.05f est la constante de temps du filtre : il définit la vitesse de correction.
    
    // DEFENSE: "Pourquoi le cap monte-t-il tout seul au démarrage ?"
    // ANSWER: Au premier cycle, on initialise l'intégrale directement sur le magnétomètre 
    // pour éviter la phase de convergence lente (20s) du filtre complémentaire.
    if (_lastUpdate == 0 || _headingIntegral == 0) {
        _headingIntegral = magHeading;
    } else {
        _headingIntegral += (gyroZRate + 0.05f * headingError) * dt;
    }
    
    // Wrap final de l'angle cumulé pour le maintenir dans l'intervalle [-PI, PI]
    while (_headingIntegral > M_PI) _headingIntegral -= 2.0f * M_PI;
    while (_headingIntegral < -M_PI) _headingIntegral += 2.0f * M_PI;

    // Mise à jour des données de navigation fusionnées
    _data.gyroZ = gyroZRate; // Vitesse de rotation corrigée
    _data.heading = _headingIntegral * (180.0f / M_PI); // Cap en degrés (0-360)
    if (_data.heading < 0) _data.heading += 360.0f;
}

void Navigation::startCalibration() {
    // Initialise les valeurs min/max pour la calibration Hard-Iron du magnétomètre
    _magMinX = 1000.0f; _magMaxX = -1000.0f;
    _magMinY = 1000.0f; _magMaxY = -1000.0f;
    _isCalibrating = true;
    _data.isCalibrated = false;
    Serial.println("[Nav] Démarrage calibration magnétomètre (rotation 360° requise).");
}

void Navigation::stopCalibration() {
    _isCalibrating = false;
    _data.isCalibrated = true;
    // Calcul des offsets finaux
    _magOffsetX = (_magMaxX + _magMinX) / 2.0f;
    _magOffsetY = (_magMaxY + _magMinY) / 2.0f;
    Serial.printf("[Nav] Calibration magnétomètre terminée. Offsets X:%.2f, Y:%.2f\n", _magOffsetX, _magOffsetY);
}

bool Navigation::isCalibrated() const {
    // Le magnétomètre est considéré calibré si les offsets ont été calculés
    return _data.isCalibrated;
}