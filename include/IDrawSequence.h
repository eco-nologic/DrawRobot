#ifndef IDRAW_SEQUENCE_H
#define IDRAW_SEQUENCE_H

#include "MotionController.h"
#include "Navigation.h"

class IDrawSequence {
public:
    virtual ~IDrawSequence() = default;
    virtual void update(MotionController& mc, Navigation& nav) = 0;
    virtual bool isFinished() const = 0;
    virtual void onFinished(Navigation& nav) {} // Hook pour le nettoyage final
};

#endif