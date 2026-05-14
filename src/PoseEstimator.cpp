#include "PoseEstimator.h"
#include <cmath>

/**
 * @brief Constructeur du gestionnaire de localisation.
 * DEFENSE: "Comment initialisez-vous la position ?"
 * ANSWER: On définit l'origine (0,0,0) au moment du démarrage, ce qui servira de référentiel pour tout le tracé.
 */
PoseEstimator::PoseEstimator(IMotor& left, IMotor& right, Navigation& nav)
    : _left(left), _right(right), _nav(nav) {}

void PoseEstimator::begin() {
    reset();
}

/**
 * @brief Calcule la nouvelle position par intégration des capteurs.
 * DEFENSE: "Comment gérez-vous la dérive de l'odométrie ?"
 * ANSWER: On utilise l'angle du magnétomètre/gyro (nav) car il est absolu et beaucoup plus précis que le calcul basé uniquement sur la différence de rotation des roues.
 */
void PoseEstimator::update() {
    // Calcul du déplacement de chaque roue depuis la dernière mise à jour
    long ticksL = _left.getTicks();
    long ticksR = _right.getTicks();
    
    float dL = (ticksL - _lastTicksL) * _mmPerTick;
    float dR = (ticksR - _lastTicksR) * _mmPerTick;
    
    _lastTicksL = ticksL;
    _lastTicksR = ticksR;

    // Distance linéaire parcourue par le centre du robot
    float dCenter = (dL + dR) / 2.0f;

    _robotPose.theta = _nav.getNavData().heading * (M_PI / 180.0f); // Conversion deg -> rad

    // Mise à jour de la position (x, y) du centre de l'essieu
    _robotPose.x += dCenter * cos(_robotPose.theta);
    _robotPose.y += dCenter * sin(_robotPose.theta);
}

/**
 * @brief Calcule la position réelle de la pointe du stylo.
 * DEFENSE: "Le robot dessine-t-il exactement ce qu'on lui demande ?"
 * ANSWER: Oui, car on projette la position du stylo qui est 130mm devant l'essieu. Sans ce calcul, les virages seraient complètement décalés sur le papier.
 */
Pose PoseEstimator::getPenPose() const {
    Pose pen;
    pen.x = _robotPose.x + Config::PEN_OFFSET * cos(_robotPose.theta);
    pen.y = _robotPose.y + Config::PEN_OFFSET * sin(_robotPose.theta);
    pen.theta = _robotPose.theta;
    return pen;
}

void PoseEstimator::reset(float x, float y, float theta) {
    _robotPose = {x, y, theta};
    _lastTicksL = _left.getTicks();
    _lastTicksR = _right.getTicks();
}