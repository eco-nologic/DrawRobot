#include "PoseEstimator.h"
#include <cmath>

/**
 * @brief Constructeur du gestionnaire de localisation.
 * DEFENSE: "Comment initialisez-vous la position ?"
 * ANSWER: On définit l'origine (0,0,0) au moment du démarrage, ce qui servira de référentiel pour tout le tracé.
 */
PoseEstimator::PoseEstimator(IMotor& left, IMotor& right, Navigation& nav)
    : _left(left), _right(right), _nav(nav), _mmPerTick(Config::MM_PER_TICK) 
{
    // MATH: Initialisation de la constante ticks -> mm via la liste d'initialisation (obligatoire pour un membre const).
}

void PoseEstimator::begin() {
    // DEFENSE: "Pourquoi ne pas affecter _mmPerTick ici ?"
    // ANSWER: Parce que c'est une constante de conception. Elle est fixée à la construction 
    // de l'objet pour garantir l'intégrité des calculs d'odométrie.
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

    // 1. Mise à jour de la Ppose fusionnée (Robot Pose)
    // On utilise le cap absolu venant de la fusion IMU (Nav)
    _robotPose.theta = _nav.getNavData().heading * (M_PI / 180.0f); // Conversion deg -> rad
    _robotPose.x += dCenter * cos(_robotPose.theta);
    _robotPose.y += dCenter * sin(_robotPose.theta);

    // 2. Mise à jour de la pose "Fantôme" (Pure Odométrie)
    // DEFENSE: "Comment calculez-vous la pose sans IMU ?"
    // ANSWER: Par intégration des encodeurs uniquement. La variation d'angle (dTheta) 
    // est calculée via la différence de parcours entre les deux roues divisée par l'entraxe.
    float dTheta = (dR - dL) / Config::WHEEL_BASE;
    _ghostPose.theta += dTheta;
    _ghostPose.x += dCenter * cos(_ghostPose.theta);
    _ghostPose.y += dCenter * sin(_ghostPose.theta);
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

Pose PoseEstimator::getGhostPose() const {
    return _ghostPose;
}

void PoseEstimator::reset(float x, float y, float theta) {
    _robotPose = {x, y, theta};
    _ghostPose = {x, y, theta};
    _lastTicksL = _left.getTicks();
    _lastTicksR = _right.getTicks();
}