#ifndef BALANCE_SEQUENCE_H
#define BALANCE_SEQUENCE_H

#include "IDrawSequence.h"
#include "PIDController.h"

class BalanceSequence : public IDrawSequence {
public:
    BalanceSequence();
    void update(MotionController& mc, Navigation& nav) override;
    bool isFinished() const override { return false; } // Mode infini jusqu'à stop
    bool isBalance() const override { return true; }

private:
    PIDController _pidBalance;
    float _fusedTilt = 0.0f;
    bool _firstRun = true;
    bool _isCalibrating = true;
    unsigned long _calibrationStartTime = 0;
    float _tiltSum = 0.0f;
    int _tiltSamples = 0;
    float _learnedOffset = 0.0f;
};

#endif