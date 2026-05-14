#include "MotionController.h"
#include <cmath>

MotionController::MotionController(DriveTrain& dt, PoseEstimator& pe)
    : _dt(dt), _pe(pe), 
      _pidLinear(Config::PidLinearKp, Config::PidLinearKi, Config::PidLinearKd),
      _pidAngular(Config::PidAngularKp, Config::PidAngularKi, Config::PidAngularKd) {
    _reached = true;
}

void MotionController::begin() {
    _pidLinear.reset();
    _pidAngular.reset();
    _reached = true;
}

void MotionController::moveTo(float x, float y) {
    _tx = x; _ty = y;
    _reached = false;
}

void MotionController::update(float dt) {
    if (_reached) return;

    // DEFENSE: "Sur quoi régulez-vous la position ?"
    // ANSWER: Sur le STYLO (Pen Pose), car c'est lui qui doit suivre le tracé précis.
    ControlErrors err = computeErrors(_pe.getPenPose());

    if (err.dist < 5.0f) { // Arrêt si on est à moins de 5mm (ET2.3)
        _dt.stop();
        _reached = true;
        return;
    }

    executeControl(err, dt);
}

MotionController::ControlErrors MotionController::computeErrors(const Pose& curr) {
    float dx = _tx - curr.x;
    float dy = _ty - curr.y;
    float dist = sqrt(dx*dx + dy*dy);

    float targetAngle = atan2(dy, dx);
    float angleErr = targetAngle - curr.theta;

    // Normalisation de l'angle [-PI, PI]
    while (angleErr > M_PI) angleErr -= 2 * M_PI;
    while (angleErr < -M_PI) angleErr += 2 * M_PI;

    return { dist, angleErr };
}

void MotionController::executeControl(const ControlErrors& err, float dt) {
    // Calcul des vitesses V (linéaire) et W (angulaire) via les PIDs
    float v = _pidLinear.compute(err.dist, 0, dt); 
    float w = _pidAngular.compute(err.angle, 0, dt);
    _dt.setVelocity(v, w); // Envoi au DriveTrain
}

void MotionController::setPidParams(float lKp, float lKi, float lKd, float aKp, float aKi, float aKd) {
    // Réinitialisation des objets PID avec les nouvelles constantes
    _pidLinear = PIDController(lKp, lKi, lKd);
    _pidAngular = PIDController(aKp, aKi, aKd);
    Serial.println("[Motion] PID parameters updated via UI.");
}