#ifndef POSE_ESTIMATOR_H
#define POSE_ESTIMATOR_H

#include "IMotor.h"
#include "Navigation.h"
#include "Config.h"

/**
 * DEFENSE: "Comment sont mesurés les angles et distances parcourues par le robot ?"
 * ANSWER: Les distances sont calculées via les encodeurs (ticks -> mm). 
 * Les angles sont calculés par le différentiel des roues et corrigés par le gyroscope (IMU).
 */
struct Pose {
    float x;      // Position X en mm
    float y;      // Position Y en mm
    float theta;  // Orientation en radians
};

class PoseEstimator {
public:
    PoseEstimator(IMotor& left, IMotor& right, Navigation& nav);

    void begin();

    /**
     * DEFENSE: "Que doit-on faire pour avoir le même système en boucle ouverte ?"
     * ANSWER: Supprimer cette classe ! En boucle ouverte, on n'utilise aucun retour 
     * (encodeur/IMU), on "espère" juste que le robot va au bon endroit.
     */
    void update();

    // Retourne la position exacte du stylo (décalage de 130mm inclus)
    Pose getPenPose() const;

    /**
     * DEFENSE: "À quoi servent les données 'Ghost' ?"
     * ANSWER: À quantifier l'apport de l'IMU. On compare la position estimée par les roues 
     * seules (Ghost) à la position fusionnée pour détecter les dérives odométriques.
     */
    Pose getGhostPose() const;

    void reset(float x = 0, float y = 0, float theta = 0);

private:
    IMotor &_left, &_right;
    Navigation &_nav;

    Pose _robotPose; // Position du centre de l'essieu
    Pose _ghostPose; // Position estimée par odométrie pure
    long _lastTicksL, _lastTicksR;
    
    // Facteurs de conversion (Config.h)
    const float _mmPerTick = Config::MM_PER_TICK;
};

#endif