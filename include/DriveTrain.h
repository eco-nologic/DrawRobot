#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "IMotor.h"
#include "Config.h"

/**
 * DEFENSE: "Faire un schéma bloc au tableau."
 * ANSWER: Le DriveTrain est le bloc 'Cinématique Inverse'. Il prend une consigne 
 * globale (Vitesse, Rotation) et calcule les consignes locales (Moteur G, Moteur D).
 */
class DriveTrain {
public:
    DriveTrain(IMotor& left, IMotor& right);

    void begin();

    /**
     * DEFENSE: "Comment pourrait-on contrôler la vitesse du robot ?"
     * ANSWER: En utilisant ce modèle différentiel. On ajuste la vitesse linéaire (V) 
     * et la vitesse angulaire (W) pour modifier la trajectoire en temps réel.
     */
    void setVelocity(float linear, float angular);

    void stop();

private:
    IMotor& _left;
    IMotor& _right;
    float _wheelBase; // Entraxe des roues (L)
};

#endif