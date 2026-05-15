#include "BalanceSequence.h"
#include "Config.h"

BalanceSequence::BalanceSequence() 
    : _pidBalance(Config::PidBalanceKp, Config::PidBalanceKi, Config::PidBalanceKd) {}

void BalanceSequence::update(MotionController& mc, Navigation& nav) {
    // DEFENSE: "Pourquoi le mode Segway ne fonctionnait pas ?"
    // ANSWER: 1. La consigne d'angle était décalée. 2. L'accéléromètre seul est trop bruité.
    // 3. La commande était trop faible pour vaincre le frottement statique (Deadband).

    // 1. Calcul de l'inclinaison (Pitch) brute via accéléromètre
    // DEFENSE: "Comment calculez-vous l'angle d'inclinaison pour l'équilibre ?"
    // ANSWER: L'axe X du robot est l'axe vertical (gravité). L'axe Z du robot est l'axe horizontal pour le tangage.
    // MATH: L'angle de Pitch est atan2(accelZ, accelX) quand le robot est debout (accelZ ~ 0, accelX ~ 9.81).
    float rawTilt = atan2(nav.getRawData().accelZ, nav.getRawData().accelX);
    
    // 2. Filtre Complémentaire local pour la stabilité dynamique
    float gyroRate = nav.getRawData().gyroY; 
    
    // MATH: Initialisation au premier cycle pour éviter la dérive de convergence.
    if (_firstRun) {
        _fusedTilt = rawTilt;
        _firstRun = false;
    }

    // MATH: On intègre le gyro et on recale sur l'accel. Alpha=0.98 privilégie le gyro à court terme.
    _fusedTilt = 0.98f * (_fusedTilt + gyroRate * 0.01f) + 0.02f * rawTilt; // dt = 0.01s (10ms)

    // 3. Calcul de l'erreur (Target = 0)
    // Si le robot tombe en avant, fusedTilt augmente. Erreur = Target - Tilt = Négatif.
    float tiltError = Config::BalancePointRad - _fusedTilt;

    // Sécurité: Si le robot tombe au delà de 45° (0.8 rad), on coupe les moteurs
    if (abs(tiltError) > 0.8f) {
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
     * MATH: L'output du PID est normalisé. On le multiplie par MaxLinearSpeed pour 
     * obtenir une consigne en mm/s.
     */
    float speed = output * Config::MaxLinearSpeedMmS; 

    // Saturation et gestion de la zone morte (Deadband)
    speed = constrain(speed, -Config::MaxLinearSpeedMmS, Config::MaxLinearSpeedMmS);

    // Deadband boost: si le PID demande un mouvement, on s'assure de dépasser 15mm/s
    // pour vaincre le frottement statique initial de l'essieu.
    if (abs(speed) > 0.1f && abs(speed) < 15.0f) {
        speed = (speed > 0) ? 15.0f : -15.0f;
    }

    /**
     * DEFENSE: "Comment le robot garde-t-il l'équilibre ?"
     * ANSWER: On injecte la sortie du PID directement dans les moteurs via getDriveTrain(). 
     * Le robot avance ou recule pour maintenir son centre de gravité verticalement 
     * au-dessus de l'essieu (modèle du pendule inversé).
     */
    mc.getDriveTrain().setVelocity(speed, 0); 
}