#ifndef SQUARES_SEQUENCE_H
#define SQUARES_SEQUENCE_H

#include "IDrawSequence.h"
#include "Config.h"

class SquaresSequence : public IDrawSequence {
public:
    SquaresSequence(float size, int count);
    void update(MotionController& mc, Navigation& nav) override;
    bool isFinished() const override;

private:
    float _size;
    int _count;
    int _subStep = 0;
};

#endif