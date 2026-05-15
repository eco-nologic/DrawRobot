#include "StairsSequence.h"

void StairsSequence::update(MotionController& mc, Navigation& nav) {
    switch (_step) {
        case 0: mc.moveTo(0, -10); _step++; break;
        case 1: mc.moveTo(0, 10); _step++; break;
        case 2: mc.moveTo(0, 0); _step++; break;
        case 3: mc.moveTo(200, 0); _targetL = 200; _targetTheta = 0; _step++; break;
        case 4: mc.moveTo(200, 100); _targetL = 100; _targetTheta = 90; _step++; break;
        case 5: mc.moveTo(600, 100); _targetL = 400; _targetTheta = 90; _step++; break;
        case 6: mc.moveTo(600, 90); _step++; break;
        case 7: mc.moveTo(600, 110); _step++; break;
    }
}

bool StairsSequence::isFinished() const {
    return _step > 7;
}