#include "BalanceSequence.h"
#include "Config.h"

BalanceSequence::BalanceSequence() 
    : _pidBalance(Config::PidBalanceKp, Config::PidBalanceKi, Config::PidBalanceKd) {}

void BalanceSequence::update(MotionController& mc, Navigation& nav) {
    // MATH: Calcul de l'inclinaison actuelle par rapport au point d'équilibre vertical
    // On récupère le Pitch calculé par la fusion de capteurs dans Navigation
    float currentPitch = nav.getRawData().accelX; // Utilisation simplifiée ou via OrientationData
    
    // En mode équilibriste, on utilise l'accéléromètre brut pour une réponse ultra-rapide
    float tiltError = Config::BalancePointRad - atan2(-nav.getRawData().accelX, nav.getRawData().accelZ);

    // Le PID calcule la vitesse moteur nécessaire pour contrer la chute
    float output = _pidBalance.compute(tiltError, 0, 0.01f); // dt = 10ms

    // On applique la même vitesse aux deux roues pour rester sur place
    // On limite pour éviter de brûler les moteurs
    float speed = constrain(output, -1.0f, 1.0f);
    
    /**
     * DEFENSE: "Comment le robot garde-t-il l'équilibre ?"
     * ANSWER: On injecte la sortie du PID directement dans les moteurs via getDriveTrain(). 
     * Le robot avance ou recule pour maintenir son centre de gravité verticalement 
     * au-dessus de l'essieu (modèle du pendule inversé).
     */
    mc.getDriveTrain().setVelocity(speed, 0); 
}