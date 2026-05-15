#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "MotionController.h"
#include "Navigation.h"

// Déclaration anticipée pour le pattern Strategy
class IDrawSequence;

/**
 * DEFENSE: "Comment est organisée la logique de tracé ?"
 * ANSWER: Par un gestionnaire de trajectoire (PathPlanner) qui décompose 
 * les formes complexes en une série de points de passage (Waypoints).
 */
class PathPlanner {
public:
    PathPlanner(MotionController& mc, Navigation& nav);

    // Démarre la Séquence 1 (Escalier)
    void startStairs();
    // Démarre la Séquence 2 (Cercle avec rayon variable - ET2.1)
    void startCircle(float radius);

    // Démarre FA1: Carrés circonscrits (Requirement FA1)
    // DEFENSE: "Comment les paramètres de taille et nombre sont-ils transmis ?"
    // ANSWER: Via le WebSocket, puis stockés dans les variables d'état de PathPlanner.
    void startSquares(float size, int count);

    // Démarre la Séquence 3 (Flèche vers le Nord - ET3.1)
    void startNorthArrow();

    // Arrête la séquence en cours
    // DEFENSE: "Comment implémentez-vous l'arrêt d'urgence ?"
    // ANSWER: En réinitialisant l'état du PathPlanner et en ordonnant au MotionController de s'arrêter.
    void stop();

    void update(float dt); 

    // Accesseur pour permettre au CommsManager de modifier les PIDs (Defense)
    MotionController& getMotionController() { return _mc; }

    // Démarre la rotation de calibration
    void startCalibrationSequence();

    // Accesseurs pour la télémétrie
    float getTargetL() const;
    float getTargetTheta() const;
    float getTargetR() const;

private:
    MotionController& _mc;
    Navigation& _nav;
    IDrawSequence* _currentSequence = nullptr;
};

#endif