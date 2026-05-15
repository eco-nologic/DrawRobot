#ifndef CALIBRATION_SEQUENCE_H
#define CALIBRATION_SEQUENCE_H

#include "IDrawSequence.h"

class CalibrationSequence : public IDrawSequence {
public:
    void update(MotionController& mc, Navigation& nav) override;
    bool isFinished() const override;

    // DEFENSE: "Comment arrêtez-vous la calibration proprement ?"
    void onFinished(Navigation& nav) override;

private:
    int _subStep = 0;
};

#endif