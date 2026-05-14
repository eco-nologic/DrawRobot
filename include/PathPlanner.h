#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include "MotionController.h"
#include "Navigation.h"

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

private:
    // --- Méthodes de découpage (Simplification pour la soutenance) ---
    void processStairsSequence();  // Gère la Séquence 1
    void processSquaresSequence(); // Gère FA1
    void processCircleSequence();  // Gère la Séquence 2
    void processNorthSequence();   // Gère la Séquence 3
    void processCalibrationSequence(); 

    // --- Sous-fonctions de tracé (Tips: Modularité) ---
    void drawMainCircle();         // Étape 100
    void drawRosacePetal();        // Étape 150
    void drawArrowBody();          // Étape 200
    void drawArrowHeadFill();      // Étape 201

    MotionController& _mc;
    Navigation& _nav;
    int _step = -1; // -1 = IDLE, >= 0 = Index de la séquence

    // Variables d'état pour les séquences
    float _targetRadius = 0;
    int _subStep = 0;
    float _currentSize = 0;
    int _currentCount = 0;
    float _arrowX = 0, _arrowY = 0; // Stockage pour la Séquence 3
};

#endif