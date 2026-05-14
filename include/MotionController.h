#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include "DriveTrain.h"
#include "PoseEstimator.h"
#include "PIDController.h"
#include "Config.h"

/**
 * DEFENSE: "Comment le robot sait-il qu'il est arrivé à destination ?"
 * ANSWER: En calculant la distance euclidienne entre le stylo et la cible. 
 * On utilise une 'tolérance' (ex: 5mm) pour valider l'arrivée (Requirement ET2.3).
 */
class MotionController {
public:
    MotionController(DriveTrain& dt, PoseEstimator& pe);

    void begin();

    // Définit une cible (X, Y) à atteindre pour le stylo
    void moveTo(float targetX, float targetY);

    // Mise à jour de la boucle d'asservissement (100Hz)
    void update(float dt);

    // Mise à jour des paramètres PID à la volée (Defense)
    void setPidParams(float lKp, float lKi, float lKd, float aKp, float aKi, float aKd);

    bool isReached() const { return _reached; }

private:
    // --- Méthodes de découpage (Logic split for Defense) ---
    struct ControlErrors { float dist; float angle; };
    ControlErrors computeErrors(const Pose& current);
    void executeControl(const ControlErrors& err, float dt);

    DriveTrain& _dt;
    PoseEstimator& _pe;
    PIDController _pidLinear, _pidAngular;
    float _tx, _ty; // Coordonnées cibles
    bool _reached = true;
};
#endif