#ifndef DCMOTOR_H
#define DCMOTOR_H

#include "IMotor.h"
#include <Arduino.h>

class DCMotor : public IMotor {
public:
    DCMotor(int pinPwm, int pinIn1, int pinIn2, int pinA, int pinB, int pwmChan);
    void begin() override;
    void setSpeed(float speed) override;
    long getTicks() const override;
    void reset() override;

private:
    void IRAM_ATTR handleInterrupt();
    int _pinPwm, _in1, _in2, _pinA, _pinB, _pwmChan;
    volatile long _ticks;
};

#endif