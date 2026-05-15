#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DS3.h>
#include <Adafruit_LIS3MDL.h>
#include "Config.h"

// Structure pour les données brutes de l'IMU
struct ImuData {
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ;
};

// Structure pour les données d'orientation fusionnées (Roll, Pitch, Heading)
struct OrientationData {
    float roll, pitch, heading;
};

/**
 * DEFENSE: "Quels protocoles sont utilisés pour communiquer avec les capteurs ?"
 * ANSWER: Le protocole I2C (Inter-Integrated Circuit) via les broches SDA(21) et SCL(22).
 */
class Navigation {
public:
    struct NavData {
        float heading;      // Cap en degrés (0-360)
        float gyroZ;        // Vitesse de rotation (deg/s)
        bool isCalibrated;  // État de la calibration magnétique
    };

    Navigation();

    // Initialise le bus I2C et les capteurs (LSM6DS3 + LIS3MDL)
    bool begin();

    /**
     * DEFENSE: "Comment sont mesurés les angles ?"
     * ANSWER: En calculant l'arc-tangente des axes X et Y du magnétomètre pour le Nord absolu, 
     * combiné à l'intégration du gyroscope pour la réactivité.
     */
    /**
     * DEFENSE: "Comment fusionnez-vous les données du gyroscope et du magnétomètre ?"
     * ANSWER: Nous utilisons un filtre complémentaire (ou Kalman) pour combiner le gyroscope (précis à court terme mais sujet à la dérive)
     * et le magnétomètre (stable à long terme mais bruité). Le gyroscope fournit la réactivité,
     * tandis que le magnétomètre corrige la dérive angulaire sur la durée.
     */
    /**
     * DEFENSE: "Comment gérez-vous l'inclinaison du robot pour le cap ?"
     * ANSWER: Nous utilisons l'accéléromètre pour calculer l'assiette (pitch et roll) du robot.
     * Ces angles sont ensuite utilisés pour compenser les lectures du magnétomètre,
     * évitant ainsi que la composante verticale du champ magnétique terrestre ne fausse le cap horizontal.
     */
    void update();

    /**
     * DEFENSE: "Pourquoi le robot doit-il rester immobile pendant le démarrage ?"
     * ANSWER: La fonction calibrate() calcule la moyenne de 200 échantillons du gyroscope. 
     * Si le robot bouge, le biais calculé sera faux, entraînant une dérive constante du cap.
     */
    void calibrate();

    /**
     * DEFENSE: "Comment compensez-vous les perturbations magnétiques ?"
     * ANSWER: Par une calibration 'Hard-Iron'. On enregistre les valeurs Min/Max 
     * sur 360° pour recentrer le cercle magnétique sur l'origine.
     */
    void startCalibration();
    void stopCalibration();
    bool isCalibrated() const;

    NavData getNavData() const { return _data; } // Données de navigation fusionnées
    ImuData getRawData() const { return rawData; } // Données brutes des capteurs
    float getHeading() const { return _data.heading; } // Cap fusionné en degrés

private:
    NavData _data;
    Adafruit_LSM6DS3 _lsm;
    Adafruit_LIS3MDL _lis;
    ImuData rawData; // Données brutes des capteurs
    OrientationData orientation; // Données d'orientation fusionnées (Roll, Pitch, Heading)
    float _magMinX = 1000, _magMaxX = -1000, _magMinY = 1000, _magMaxY = -1000;
    float _gyroBiasZ = 0;
    float _headingIntegral = 0;
    unsigned long _lastUpdate = 0;
    bool _isCalibrating = false;
    float _magOffsetX = 0.0f; 
    float _magOffsetY = 0.0f;
    float _magOffsetZ = 0.0f;
};

#endif