#include "DriveTrain.h"
#include "Config.h"
#include <Arduino.h>

/**
 * @brief Constructeur de la plateforme différentielle.
 * DEFENSE: "Pourquoi utiliser une plateforme à deux roues motrices ?"
 * ANSWER: C'est le modèle le plus simple pour le tracé (pivots sur place possibles), permettant une grande agilité pour dessiner des angles droits.
 */
DriveTrain::DriveTrain(IMotor& left, IMotor& right) 
    : _left(left), _right(right), _wheelBase(Config::WHEEL_BASE) {}

void DriveTrain::begin() {
    _left.begin();
    _right.begin();
}

/**
 * @brief Traduit une commande globale en commandes moteurs locales.
 * DEFENSE: "Quel est le système de coordonnées utilisé ?"
 * ANSWER: Pour le déplacement, on utilise la cinématique différentielle : vL = V - (W*L/2) et vR = V + (W*L/2).
 */
void DriveTrain::setVelocity(float linear, float angular) {
    // Calcul des vitesses individuelles des roues
    // angular est en rad/s, linear est normalisé ou en mm/s
    float leftVel  = linear - (angular * _wheelBase / 2.0f);
    float rightVel = linear + (angular * _wheelBase / 2.0f);

    // Contrainte des valeurs pour éviter la saturation logicielle
    leftVel  = constrain(leftVel, -1.0f, 1.0f);
    rightVel = constrain(rightVel, -1.0f, 1.0f);

    _left.setSpeed(leftVel);
    _right.setSpeed(rightVel);
}

/**
 * DEFENSE: "Pourquoi utiliser un système de coordonnées polaires ?"
 * ANSWER: C'est l'essence du robot différentiel. On définit un angle (Cap) 
 * et une distance, ce qui correspond directement aux commandes W (angulaire) et V (linéaire).
 */
void DriveTrain::stop() {
    _left.setSpeed(0);
    _right.setSpeed(0);
}