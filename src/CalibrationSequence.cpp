#include "CalibrationSequence.h"
#include <cmath>

void CalibrationSequence::update(MotionController& mc, Navigation& nav) {
    float angle = _subStep * (M_PI / 4.0f);
    // Move to 8 points around a small 20mm circle to rotate the frame
    mc.moveTo(20.0f * cos(angle), 20.0f * sin(angle));
    _subStep++;
}

bool CalibrationSequence::isFinished() const {
    return _subStep >= 8;
}

void CalibrationSequence::onFinished(Navigation& nav) {
    // ANSWER: On laisse la séquence notifier elle-même la fin du mode calibration.
    nav.stopCalibration();
}