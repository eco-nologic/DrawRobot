#ifndef NORTH_SEQUENCE_H
#define NORTH_SEQUENCE_H

#include "IDrawSequence.h"

class NorthSequence : public IDrawSequence {
public:
    void update(MotionController& mc, Navigation& nav) override;
    bool isFinished() const override;

private:
    int _subStep = 0;
    float _ax = 0, _ay = 0;
};

#endif