#include "DCMotor.h"
#include "Config.h"

/**
 * @brief Constructeur du pilote moteur DC.
 * DEFENSE: "Comment gérez-vous l'instanciation des moteurs ?"
 * ANSWER: Par injection de dépendance des pins matérielles définies dans Config.h pour découpler le code du hardware.
 */
DCMotor::DCMotor(int pinPwm, int pinIn1, int pinIn2, int pinA, int pinB, int pwmChan)
    : _pinPwm(pinPwm), _in1(pinIn1), _in2(pinIn2), _pinA(pinA), _pinB(pinB), _pwmChan(pwmChan), _ticks(0) {
}

/**
 * DEFENSE: "Comment fonctionne l’encodeur ?"
 * ANSWER: En utilisant le décodage en quadrature sur interruption (ISR). 
 * On détecte le déphasage entre Canal A et B pour connaître le sens.
 */
void IRAM_ATTR DCMotor::handleInterrupt() {
    // Read current state of B when A changes
    if (digitalRead(_pinA) == digitalRead(_pinB)) _ticks++;
    else _ticks--;
}

/**
 * @brief Initialise les périphériques GPIO et PWM pour le moteur.
 * DEFENSE: "Quelle est la forme des signaux envoyés ?"
 * ANSWER: Des signaux PWM (Pulse Width Modulation) générés par le périphérique LEDC de l'ESP32 à 5kHz.
 */
void DCMotor::begin() {
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    
    // Attach interrupt to Phase A for high-speed tracking
    attachInterruptArg(digitalPinToInterrupt(_pinA), 
                       [](void* arg) { static_cast<DCMotor*>(arg)->handleInterrupt(); }, 
                       this, CHANGE);

    // Setup PWM using ESP32 ledc
    ledcSetup(_pwmChan, Config::PWM_FREQ, Config::PWM_RES);
    ledcAttachPin(_pinPwm, _pwmChan);
}

void DCMotor::setSpeed(float speed) {
    // DEFENSE: "Comment pourrait-on contrôler la vitesse du robot ?"
    // ANSWER: En mappant la vitesse cible sur une valeur PWM (Pulse Width Modulation).
    
    bool forward = speed >= 0;
    float absSpeed = abs(speed);
    
    // Handle Deadband (ET1: Static friction compensation)
    uint32_t duty = 0;
    if (absSpeed > 0.01f) {
        duty = Config::DEADBAND + (absSpeed * (255 - Config::DEADBAND));
    }

    digitalWrite(_in1, forward ? HIGH : LOW);
    digitalWrite(_in2, forward ? LOW : HIGH);
    ledcWrite(_pwmChan, duty);
}

/**
 * DEFENSE: "Que faire pour avoir le même système en boucle ouverte ?"
 * ANSWER: Retirer l'utilisation des ticks d'encodeurs et ne se fier qu'à la tension (PWM).
 * C'est beaucoup moins précis car la batterie et la friction varient.
 */
long DCMotor::getTicks() const { return _ticks; }

void DCMotor::reset() { _ticks = 0; }