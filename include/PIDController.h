#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/**
 * DEFENSE: "Faire un schéma bloc au tableau."
 * ANSWER: [Consigne] -> (+) -> [Erreur] -> [PID] -> [Moteur] -> [Système] --(Mesure)--> (-)
 * Le PID calcule une correction pour réduire l'erreur entre la consigne et la mesure.
 */
class PIDController {
public:
    PIDController(float kp, float ki, float kd);

    // dt: temps écoulé en secondes pour l'intégration/dérivation
    float compute(float setpoint, float measured, float dt);

    void reset();

private:
    float _kp, _ki, _kd;
    float _integral = 0;
    float _lastError = 0;
};

#endif