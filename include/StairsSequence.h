#ifndef STAIRS_SEQUENCE_H
#define STAIRS_SEQUENCE_H

#include "IDrawSequence.h"

class StairsSequence : public IDrawSequence {
public:
    void update(MotionController& mc, Navigation& nav) override;
    bool isFinished() const override;

private:
    int _step = 0;
};

#endif