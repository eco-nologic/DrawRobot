#include "PIDController.h"
#include <Arduino.h>

PIDController::PIDController(float kp, float ki, float kd) 
    : _kp(kp), _ki(ki), _kd(kd) {}

/**
 * @brief Calcule la sortie du correcteur PID.
 * DEFENSE: "Comment contrôlez-vous la précision du tracé ?"
 * ANSWER: Via un double asservissement PID (linéaire et angulaire) qui minimise l'erreur entre la pose estimée et la cible.
 */
float PIDController::compute(float setpoint, float measured, float dt) {
    float error = setpoint - measured;

    /**
     * DEFENSE: "À quoi sert le terme Intégral ?"
     * ANSWER: À éliminer l'erreur statique. Sans 'I', les frottements mécaniques 
     * empêchent le robot d'atteindre exactement 20cm; il s'arrêterait à 19.5cm.
     */
    _integral += error * dt;

    // DEFENSE: "Comment gérez-vous l'emballement de l'intégrale (Windup) ?"
    // ANSWER: Nous limitons (clamping) la valeur de l'intégrale. Si le robot est bloqué, 
    // cela évite qu'il n'accumule trop d'énergie et ne "saute" une fois libéré.
    if (_integral > 255.0f) _integral = 255.0f;
    if (_integral < -255.0f) _integral = -255.0f;

    /**
     * DEFENSE: "Pourquoi utiliser le terme Dérivé ?"
     * ANSWER: Pour anticiper l'approche de la consigne et éviter de dépasser 
     * la cible (overshoot) en "freinant" logiciellement.
     */
    float derivative = (error - _lastError) / dt;
    _lastError = error;

    return (_kp * error) + (_ki * _integral) + (_kd * derivative);
}

void PIDController::reset() {
    _integral = 0;
    _lastError = 0;
}