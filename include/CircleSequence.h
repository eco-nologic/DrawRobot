#ifndef CIRCLE_SEQUENCE_H
#define CIRCLE_SEQUENCE_H

#include "IDrawSequence.h"
#include "Config.h"

class CircleSequence : public IDrawSequence {
public:
    CircleSequence(float radius);
    void update(MotionController& mc, Navigation& nav) override;
    bool isFinished() const override;

    float getTargetR() const override { return _radius; }

private:
    float _radius;
    int _subStep = 0;
    bool _drawingRosace = false;
    int _petalIndex = 0;
};

#endif