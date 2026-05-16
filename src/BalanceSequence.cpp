#include "BalanceSequence.h"
#include "Config.h"

BalanceSequence::BalanceSequence() 
    : _pidBalance(Config::PidBalanceKp, Config::PidBalanceKi, Config::PidBalanceKd) {}

void BalanceSequence::update(MotionController& mc, Navigation& nav) {
    // DEFENSE: "Pourquoi le mode Segway ne fonctionnait pas ?"
    // ANSWER: 1. La consigne d'angle était décalée. 2. L'accéléromètre seul est trop bruité.
    // 3. La commande était trop faible pour vaincre le frottement statique (Deadband).

    // 1. Mesure brute de l'inclinaison (Pitch) par accéléromètre
    // DEFENSE: "Comment calculez-vous l'angle d'inclinaison pour l'équilibre ?"
    // ANSWER: L'axe X du robot est l'axe vertical (gravité). L'axe Z du robot est l'axe horizontal pour le tangage.
    // MATH: L'angle de Pitch est atan2(accelZ, accelX) quand le robot est debout (accelZ ~ 0, accelX ~ 9.81).
    float rawTilt = atan2(nav.getRawData().accelZ, nav.getRawData().accelX);
    
    // 2. Filtre Complémentaire (Revert to working version)
    // DEFENSE: "Pourquoi être revenu au filtre complémentaire ?"
    // ANSWER: Le filtre de Kalman introduisait trop de latence de calcul. Le filtre 
    // complémentaire est plus réactif pour un système instable à 100Hz.
    float gyroRate = nav.getRawData().gyroY; 

    if (_firstRun) {
        _fusedTilt = rawTilt;
        _firstRun = false;
        _calibrationStartTime = millis();
        _tiltSum = 0;
        _tiltSamples = 0;
        _isCalibrating = true;
        Serial.println("[Segway] 🤝 Apprentissage : Tenez le robot parfaitement vertical...");
    }

    // MATH: Alpha=0.98 privilégie le gyro (réactivité) et recale sur l'accel (stabilité).
    _fusedTilt = 0.98f * (_fusedTilt + gyroRate * 0.01f) + 0.02f * rawTilt;

    // Phase de calibration Prentice (Apprentissage de l'assiette verticale)
    if (_isCalibrating) {
        _tiltSum += _fusedTilt;
        _tiltSamples++;
        
        // On accumule les données pendant 3 secondes
        if (millis() - _calibrationStartTime > 3000) {
            _learnedOffset = _tiltSum / _tiltSamples;
            _isCalibrating = false;
            Serial.printf("[Segway] ✅ Apprentissage terminé. Offset : %.3f rad\n", _learnedOffset);
        } else {
            // On force l'arrêt des moteurs pendant que l'humain règle la position
            mc.getDriveTrain().stop(); 
            return; 
        }
    }

    // 3. Calcul de l'erreur
    // MATH: On utilise l'offset appris dynamiquement au lieu de la constante statique
    float tiltError = _learnedOffset - _fusedTilt;

    // Sécurité: Si le robot tombe au delà de 35° (environ 20 degrés), on coupe les moteurs
    if (abs(tiltError) > 0.6f) {
        mc.getDriveTrain().stop();
        _fusedTilt = 0;
        _firstRun = true;
        return;
    }

    // 4. Calcul du PID et scaling
    // L'output est une consigne de vitesse. On la multiplie par MaxLinearSpeed 
    // pour permettre au robot d'utiliser toute la dynamique moteur.
    float output = _pidBalance.compute(tiltError, 0, 0.01f); 

    /**
     * MATH: On réduit le multiplicateur de vitesse (80 au lieu de 120) pour rendre 
     * le robot moins "nerveux" et permettre un cruise plus fluide ("slowly").
     */
    float speed = output * 80.0f; 

    // Saturation et gestion de la zone morte (Deadband)
    speed = constrain(speed, -80.0f, 80.0f);

    // Deadband boost réduit pour un cruise plus doux près de la verticale.
    if (abs(speed) > 0.1f && abs(speed) < 10.0f) {
        speed = (speed > 0) ? 10.0f : -10.0f;
    }

    /**
     * DEFENSE: "Comment le robot garde-t-il l'équilibre ?"
     * ANSWER: On injecte la sortie du PID directement dans les moteurs via getDriveTrain(). 
     * Le robot avance ou recule pour maintenir son centre de gravité verticalement 
     * au-dessus de l'essieu (modèle du pendule inversé).
     */
    mc.getDriveTrain().setVelocity(speed, 0); 
}