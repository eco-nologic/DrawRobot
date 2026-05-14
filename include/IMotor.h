#ifndef IMOTOR_H
#define IMOTOR_H

/**
 * DEFENSE: "Comment fonctionne l'abstraction matérielle ?"
 * ANSWER: Via l'héritage C++. L'interface définit le 'contrat' de contrôle.
 */
class IMotor {
public:
    virtual ~IMotor() = default;
    virtual void begin() = 0;
    // speed: -1.0 to 1.0 (float for precision optimization)
    virtual void setSpeed(float speed) = 0; 
    virtual long getTicks() const = 0;
    virtual void reset() = 0;
};

#endif