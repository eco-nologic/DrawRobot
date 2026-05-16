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

    /**
     * @brief Permet d'identifier le mode équilibre pour optimiser les ressources.
     */
    virtual bool isBalance() const { return false; }

    // MATH: Accesseurs pour les valeurs théoriques demandées par travail.pdf
    virtual float getTargetL() const { return 0.0f; }
    virtual float getTargetTheta() const { return 0.0f; }
    virtual float getTargetR() const { return 0.0f; }
};

#endif