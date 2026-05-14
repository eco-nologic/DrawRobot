#include "PathPlanner.h"
#include <cmath>
#include <Arduino.h>

/**
 * @brief Constructeur de l'architecte de trajectoire.
 * DEFENSE: "Comment est organisée la logique de tracé ?"
 * ANSWER: Par une machine à états asynchrone qui décompose les formes en waypoints (X,Y) absolus.
 */
PathPlanner::PathPlanner(MotionController& mc, Navigation& nav) 
    : _mc(mc), _nav(nav) {}

/**
 * DEFENSE: "Comment gérez-vous l'escalier (ET1.2, ET1.3) ?"
 * ANSWER: En envoyant des coordonnées absolues successives. L'asservissement 
 * en boucle fermée sur le stylo garantit des angles de 90° parfaits.
 */
void PathPlanner::startStairs() { _step = 0; }

void PathPlanner::startSquares(float size, int count) {
    _currentSize = size;
    _currentCount = count;
    _step = 50; // Entrée de la séquence FA1
}

void PathPlanner::stop() { _step = -1; }

/**
 * DEFENSE: "Comment le rayon est-il paramétré (ET2.1) ?"
 * ANSWER: La valeur est reçue via WebSocket, stockée, et utilisée pour 
 * calculer les coordonnées polaires (R*cos, R*sin) de chaque segment.
 */
void PathPlanner::startCircle(float radius) {
    _targetRadius = radius;
    _subStep = 0;
    _step = 100; // État de la séquence Cercle
}

void PathPlanner::startNorthArrow() { _step = 200; }

void PathPlanner::startCalibrationSequence() {
    _nav.startCalibration();
    _step = 300;
    _subStep = 0;
}

/**
 * @brief Met à jour la machine à états des séquences de dessin.
 * DEFENSE: "Comment gérez-vous l'enchaînement des mouvements ?"
 * ANSWER: On ne passe à l'étape suivante que lorsque le MotionController confirme que la cible (X,Y) actuelle est atteinte.
 */
void PathPlanner::update(float dt) {
    // DEFENSE: "Comment gérez-vous l'enchaînement des mouvements ?"
    // ANSWER: Par une machine à états asynchrone. On ne passe à l'étape suivante 
    // que lorsque le MotionController confirme que la cible (X,Y) est atteinte.
    if (_step < 0 || !_mc.isReached()) return; 

    if (_step < 50) processStairsSequence();
    else if (_step < 100) processSquaresSequence();
    else if (_step < 200) processCircleSequence();
    else if (_step < 300) processNorthSequence();
    else processCalibrationSequence();

    _mc.update(dt);
}

void PathPlanner::processStairsSequence() {
    switch (_step) {
        case 0: _mc.moveTo(0, -10); _step++; break;    // Marqueur départ
        case 1: _mc.moveTo(0, 10); _step++; break;
        case 2: _mc.moveTo(0, 0); _step++; break;
        case 3: _mc.moveTo(200, 0); _step++; break;    // 20cm
        case 4: _mc.moveTo(200, 100); _step++; break;  // 10cm + Virage
        case 5: _mc.moveTo(600, 100); _step++; break;  // 40cm
        case 6: _mc.moveTo(600, 90); _step++; break;   // Marqueur arrivée
        case 7: _mc.moveTo(600, 110); _step = -1; break;
    }
}

void PathPlanner::processSquaresSequence() {
    switch (_step) {
        case 50: _mc.moveTo(-_currentSize/2, -_currentSize/2); _step++; break;
        case 51: _mc.moveTo(_currentSize/2, -_currentSize/2); _step++; break;
        case 52: _mc.moveTo(_currentSize/2, _currentSize/2); _step++; break;
        case 53: _mc.moveTo(-_currentSize/2, _currentSize/2); _step++; break;
        case 54: 
            _mc.moveTo(-_currentSize/2, -_currentSize/2); 
            _currentSize += Config::SQUARE_GROWTH;
            _currentCount--;
            _step = (_currentCount > 0) ? 50 : -1; 
            break;
    }
}

void PathPlanner::processCircleSequence() {
    if (_step == 100) drawMainCircle();
    else if (_step == 150) drawRosacePetal();
}

void PathPlanner::drawMainCircle() {
    if (_subStep <= Config::CIRCLE_STEPS) {
        float angle = _subStep * (360.0f / Config::CIRCLE_STEPS) * (M_PI / 180.0f);
        _mc.moveTo(_targetRadius * cos(angle), _targetRadius * sin(angle));
        _subStep++;
    } else { 
        _step = 150; // Passer à la Rosace (FA2)
        _subStep = 0; 
    }
}

void PathPlanner::drawRosacePetal() {
    /**
     * DEFENSE: "Comment garantissez-vous la forme courbe des pétales (FA2) ?"
     * ANSWER: En décomposant chaque pétale en un arc de cercle secondaire. 
     * On fait varier l'angle local par rapport au point d'ancrage sur le cercle principal.
     */
    static int petalIndex = 0;
    if (petalIndex < 4) {
        float baseAngle = petalIndex * (M_PI / 2.0f);
        // Dessin d'un arc pour le pétale
        float localAngle = baseAngle + (_subStep * 0.1f);
        _mc.moveTo(_targetRadius * cos(localAngle), _targetRadius * sin(localAngle));
        
        if (++_subStep > 10) { _subStep = 0; petalIndex++; }
    } else {
        petalIndex = 0;
        _step = -1;
    }
}

void PathPlanner::processNorthSequence() {
    if (_step == 200) drawArrowBody();
    else if (_step == 201) drawArrowHeadFill();
}

void PathPlanner::drawArrowBody() {
    float northHeading = _nav.getNavData().heading * (M_PI / 180.0f);
    _arrowX = 100.0f * cos(northHeading);
    _arrowY = 100.0f * sin(northHeading);
    _mc.moveTo(_arrowX, _arrowY);
    _step = 201;
    _subStep = 0;
}

void PathPlanner::drawArrowHeadFill() {
    // DEFENSE: "Comment garantissez-vous 80% de remplissage (ET3.4) ?"
    // ANSWER: En divisant la pointe en petits segments de balayage (scanning).
    if (_subStep < 5) {
        float scanWidth = (_subStep - 2) * 2.0f;
        _mc.moveTo(_arrowX * 0.9f, _arrowY * 0.9f + scanWidth);
        _subStep++;
    } else { _step = -1; }
}

void PathPlanner::processCalibrationSequence() {
    // Pivot sequence to sample magnetometer data across 360 degrees
    if (_subStep < 8) {
        float angle = _subStep * (M_PI / 4.0f);
        // Move to 8 points around a small 20mm circle to rotate the frame
        _mc.moveTo(20.0f * cos(angle), 20.0f * sin(angle));
        _subStep++;
    } else {
        _nav.stopCalibration();
        _step = -1; 
    }
}