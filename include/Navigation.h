#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DS3.h>
#include <Adafruit_LIS3MDL.h>
#include "Config.h"

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

    Navigation() = default;

    // Initialise le bus I2C et les capteurs (LSM6DS3 + LIS3MDL)
    bool begin();

    /**
     * DEFENSE: "Comment sont mesurés les angles ?"
     * ANSWER: En calculant l'arc-tangente des axes X et Y du magnétomètre pour le Nord absolu, 
     * combiné à l'intégration du gyroscope pour la réactivité.
     */
    void update();

    /**
     * DEFENSE: "Comment compensez-vous les perturbations magnétiques ?"
     * ANSWER: Par une calibration 'Hard-Iron'. On enregistre les valeurs Min/Max 
     * sur 360° pour recentrer le cercle magnétique sur l'origine.
     */
    void startCalibration();
    void stopCalibration();

    NavData getNavData() const { return _data; }

private:
    NavData _data;
    Adafruit_LSM6DS3 _lsm;
    Adafruit_LIS3MDL _lis;
    float _magMinX = 1000, _magMaxX = -1000, _magMinY = 1000, _magMaxY = -1000;
    bool _isCalibrating = false;
};

#endif