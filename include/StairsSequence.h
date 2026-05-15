#ifndef STAIRS_SEQUENCE_H
#define STAIRS_SEQUENCE_H

#include "IDrawSequence.h"

class StairsSequence : public IDrawSequence {
public:
    void update(MotionController& mc, Navigation& nav) override;
    bool isFinished() const override;

    float getTargetL() const override { return _targetL; }
    float getTargetTheta() const override { return _targetTheta; }

private:
    int _step = 0;
    float _targetL = 0;
    float _targetTheta = 0;
};

#endif