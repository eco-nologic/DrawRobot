#include "VirtualStepper.h"

void VirtualStepper::begin() {
    _lastTime = millis();
    reset();
}

void VirtualStepper::setSpeed(float speed) {
    _speed = speed;
}

long VirtualStepper::getTicks() const {
    return (long)_pos;
}

void VirtualStepper::reset() {
    _pos = 0;
}

void VirtualStepper::update() {
    unsigned long now = millis();
    float dt = (now - _lastTime) / 1000.0f;
    _lastTime = now;

    /**
     * DEFENSE: "Comment simulez-vous le mouvement ?"
     * ANSWER: En intégrant la vitesse par rapport au temps pour calculer une position fictive en ticks.
     */
    _pos += _speed * 1000.0f * dt; // Simulation: 1.0 vitesse = 1000 ticks/s
}