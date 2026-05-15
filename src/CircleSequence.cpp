#include "CircleSequence.h"
#include <cmath>

CircleSequence::CircleSequence(float radius) : _radius(radius) {}

void CircleSequence::update(MotionController& mc, Navigation& nav) {
    if (!_drawingRosace) {
        if (_subStep <= Config::CIRCLE_STEPS) {
            float angle = _subStep * (360.0f / Config::CIRCLE_STEPS) * (M_PI / 180.0f);
            mc.moveTo(_radius * cos(angle), _radius * sin(angle));
            _subStep++;
        } else {
            _drawingRosace = true;
            _subStep = 0;
        }
    } else {
        float baseAngle = _petalIndex * (M_PI / 2.0f);
        // Dessin d'un arc pour le pétale
        float localAngle = baseAngle + (_subStep * 0.1f);
        mc.moveTo(_radius * cos(localAngle), _radius * sin(localAngle));
        
        if (++_subStep > 10) { _subStep = 0; _petalIndex++; }
    }
}

bool CircleSequence::isFinished() const {
    return _petalIndex >= 4;
}