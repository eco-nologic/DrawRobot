#ifndef VIRTUAL_STEPPER_H
#define VIRTUAL_STEPPER_H

#include "IMotor.h"
#include <Arduino.h>

/**
 * @class VirtualStepper
 * @brief Simulateur logiciel de moteur pour test sans matériel
 */
class VirtualStepper : public IMotor {
public:
    void begin() override;
    void setSpeed(float speed) override;
    long getTicks() const override;
    void reset() override;
    void update(); // Mise à jour de la physique simulée

private:
    float _speed = 0;
    float _pos = 0;
    unsigned long _lastTime = 0;
};

#endif