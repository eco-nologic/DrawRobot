#include "PathPlanner.h"
#include <cmath>
#include <Arduino.h>
#include "IDrawSequence.h"
#include "StairsSequence.h"
#include "SquaresSequence.h"
#include "CircleSequence.h"
#include "NorthSequence.h"
#include "CalibrationSequence.h"

/**
 * DEFENSE: "Pourquoi utiliser des classes différentes pour chaque tracé ?"
 * ANSWER: Pour appliquer le principe de responsabilité unique. Chaque classe encapsule 
 * sa propre logique de génération de waypoints, rendant le code plus modulaire et 
 * facilitant l'ajout de nouvelles formes (ex: Polygones, Spirales).
 */

// Implementation de PathPlanner

PathPlanner::PathPlanner(MotionController& mc, Navigation& nav) 
    : _mc(mc), _nav(nav), _currentSequence(nullptr) {}

void PathPlanner::startStairs() { 
    stop();
    _currentSequence = new StairsSequence();
}

void PathPlanner::startSquares(float size, int count) {
    stop();
    _currentSequence = new SquaresSequence(size, count);
}

void PathPlanner::stop() { 
    if (_currentSequence) {
        delete _currentSequence;
        _currentSequence = nullptr;
    }
}

void PathPlanner::startCircle(float radius) {
    stop();
    _currentSequence = new CircleSequence(radius);
}

void PathPlanner::startNorthArrow() { 
    stop();
    _currentSequence = new NorthSequence();
}

void PathPlanner::startCalibrationSequence() {
    stop();
    _nav.startCalibration();
    _currentSequence = new CalibrationSequence();
}

void PathPlanner::update(float dt) {
    if (!_currentSequence) return; 

    // On ne calcule le prochain waypoint que si le précédent est atteint
    if (_mc.isReached()) {
        if (_currentSequence->isFinished()) {
            // DEFENSE: "Pourquoi ne pas utiliser dynamic_cast pour détecter la fin de calibration ?"
            // ANSWER: dynamic_cast est lent et nécessite le RTTI. Nous utilisons un 'Hook' virtuel 
            // onFinished(). C'est un appel via vtable (O(1)), bien plus performant en temps réel.
            _currentSequence->onFinished(_nav);
            
            stop();
            return;
        }
        _currentSequence->update(_mc, _nav);
    }
}

float PathPlanner::getTargetL() const {
    return _currentSequence ? _currentSequence->getTargetL() : 0.0f;
}

float PathPlanner::getTargetTheta() const {
    return _currentSequence ? _currentSequence->getTargetTheta() : 0.0f;
}

float PathPlanner::getTargetR() const {
    return _currentSequence ? _currentSequence->getTargetR() : 0.0f;
}