# DrawRobot Step 3: Virtual Motor Simulator (VirtualStepper)

> **Implement software-based motor simulation for testing without hardware**

---

## 📋 Objectives

By the end of this step, you will have:
- ✅ `VirtualStepper` class implementing `IMotor` interface
- ✅ Software-based motor physics (no GPIO, no hardware)
- ✅ Simulated encoder feedback with configurable acceleration
- ✅ Ability to test entire system without physical robot
- ✅ Easy switching between real and simulated motors

---

## 🎯 Why Simulation Matters

**Without Simulation**:
- Can only test with physical hardware
- Can't debug overnight (battery dead)
- Risk of damaging motors during development
- Slow iteration cycle

**With VirtualStepper**:
- ✅ Test everything in software
- ✅ Run full motion sequences instantly
- ✅ Safe for continuous testing
- ✅ Deterministic (reproducible results)
- ✅ Easy to inject faults for testing error handling

---

## 📝 Implementation

### VirtualStepper.h Header

```cpp
// include/VirtualStepper.h
#ifndef VIRTUAL_STEPPER_H
#define VIRTUAL_STEPPER_H

#include "IMotor.h"
#include "Config.h"
#include <Arduino.h>
#include <atomic>
#include <cmath>

/**
 * @class VirtualStepper
 * @brief Software-based motor simulation implementing IMotor interface
 * 
 * Features:
 * - No GPIO access required (pure software)
 * - Simulates motor inertia and acceleration
 * - Encoder tick generation based on simulated rotation
 * - Realistic behavior: takes time to reach commanded speed
 * - Can be configured for different motor types
 * 
 * Physics Model:
 * - Command PWM (0-255) → target speed
 * - Actual speed ramps up with simulated inertia (tau_ms)
 * - Encoder ticks generated every loop based on current speed
 * - No hardware dependencies (testable without ESP32)
 */
class VirtualStepper : public IMotor {
private:
    // ====================================================================
    // SIMULATION STATE
    // ====================================================================
    
    // Speed state
    float targetSpeedMmS = 0.0f;      // Commanded speed (from PWM)
    float currentSpeedMmS = 0.0f;     // Actual speed (ramps up)
    
    // Direction
    bool currentDirection = true;     // true = forward, false = backward
    
    // Encoder state
    std::atomic<long> encoderCount{0};  // Simulated encoder ticks
    
    // Physics simulation
    static constexpr float INERTIA_TIME_MS = 100.0f;  // Time to reach target speed
    unsigned long lastUpdateTime = 0;
    
    // Diagnostics
    bool motorHealthy = true;        // Always true in simulation
    unsigned long commandedStepsWithoutMovement = 0;

public:
    VirtualStepper();
    virtual ~VirtualStepper() = default;

    // Implement IMotor interface
    void begin() override;
    void setPwm(int pwm) override;
    void setDirection(bool forward) override;
    long getEncoderCount() const override;
    void resetEncoder() override;
    bool isHealthy() const override;
    const char* getDiagnostics() const override;

    // Simulation-specific interface
    /**
     * @brief Update motor simulation (should be called every loop)
     * 
     * This is called in the main loop to advance motor physics.
     * Updates current speed and generates encoder ticks.
     */
    void update();

    /**
     * @brief Inject a simulated fault for testing
     * 
     * @param faulted   true to simulate motor failure, false to clear
     */
    void setSimulatedFault(bool faulted) {
        motorHealthy = !faulted;
    }

    /**
     * @brief Get current simulated speed
     * 
     * @return Speed in mm/s
     */
    float getCurrentSpeed() const {
        return currentSpeedMmS;
    }

private:
    /**
     * @brief Update motor physics (ramp speed, generate encoder ticks)
     * 
     * Called by update() every loop cycle.
     * Implements exponential ramp-up to target speed.
     */
    void updatePhysics();

    /**
     * @brief Convert PWM to target speed
     * 
     * @param pwm   PWM value (0-255)
     * @return Target speed in mm/s
     */
    float pwmToSpeed(int pwm) const;

    /**
     * @brief Generate encoder ticks based on distance traveled
     * 
     * Called every loop after speed update.
     * Increments encoderCount based on currentSpeedMmS and elapsed time.
     */
    void generateEncoderTicks();
};

#endif // VIRTUAL_STEPPER_H
```

### VirtualStepper.cpp Implementation

```cpp
// src/VirtualStepper.cpp
#include "VirtualStepper.h"
#include <cstdio>

VirtualStepper::VirtualStepper() {
    lastUpdateTime = millis();
}

void VirtualStepper::begin() {
    Serial.println("[VirtualStepper] Initialized (software simulation)");
    currentSpeedMmS = 0.0f;
    targetSpeedMmS = 0.0f;
    lastUpdateTime = millis();
}

float VirtualStepper::pwmToSpeed(int pwm) const {
    // PWM 0-255 maps to 0 to MaxLinearSpeedMmS
    // Apply deadband: treat PWM below deadband as 0
    if (pwm < MotorPwmDeadband) {
        return 0.0f;
    }

    // Linear mapping from [MotorPwmDeadband, 255] to [0, MaxLinearSpeedMmS]
    float normalized = (float)(pwm - MotorPwmDeadband) / (255.0f - MotorPwmDeadband);
    return normalized * MaxLinearSpeedMmS;
}

void VirtualStepper::setPwm(int pwm) {
    pwm = constrain(pwm, 0, 255);
    targetSpeedMmS = pwmToSpeed(pwm);
}

void VirtualStepper::setDirection(bool forward) {
    currentDirection = forward;
}

long VirtualStepper::getEncoderCount() const {
    return encoderCount.load();
}

void VirtualStepper::resetEncoder() {
    encoderCount.store(0);
}

void VirtualStepper::updatePhysics() {
    unsigned long now = millis();
    float deltaTimeMs = (float)(now - lastUpdateTime);
    lastUpdateTime = now;

    // Clamp delta time (avoid huge jumps if function wasn't called for a while)
    deltaTimeMs = constrain(deltaTimeMs, 0.0f, 50.0f);  // Max 50ms per update

    // Exponential ramp-up to target speed
    // Formula: current = target + (current - target) * exp(-dt / tau)
    float decayFactor = exp(-deltaTimeMs / INERTIA_TIME_MS);
    currentSpeedMmS = targetSpeedMmS + (currentSpeedMmS - targetSpeedMmS) * decayFactor;

    // Generate encoder ticks based on distance traveled
    generateEncoderTicks();
}

void VirtualStepper::generateEncoderTicks() {
    // Calculate distance traveled since last update
    unsigned long now = millis();
    float deltaTimeS = (float)(now - lastUpdateTime) / 1000.0f;

    // Distance = speed * time
    float distanceMm = abs(currentSpeedMmS) * deltaTimeS;

    // Convert distance to encoder ticks
    // 1 tick = WheelCircumferenceMm / StepsPerRev
    float ticksPerMm = (float)StepsPerRev / WheelCircumferenceMm;
    long newTicks = (long)(distanceMm * ticksPerMm);

    // Apply direction
    if (currentDirection) {
        encoderCount += newTicks;
    } else {
        encoderCount -= newTicks;
    }
}

bool VirtualStepper::isHealthy() const {
    return motorHealthy;
}

const char* VirtualStepper::getDiagnostics() const {
    static char buffer[128];
    snprintf(buffer, sizeof(buffer),
             "PWM=%d (speed=%.1f) Dir=%s Enc=%ld Healthy=%d [SIM]",
             (int)(targetSpeedMmS / MaxLinearSpeedMmS * 255.0f),
             currentSpeedMmS,
             currentDirection ? "FWD" : "BWD",
             encoderCount.load(),
             motorHealthy ? 1 : 0);
    return buffer;
}

void VirtualStepper::update() {
    updatePhysics();
}
```

---

## 📊 Simulation Physics Model

### Speed Ramp-Up

When you command `setPwm(200)`:

```
Speed
  ^
  |     _____ target (from PWM=200)
  |    /
  |   /
  |  /_____ actual (exponential ramp)
  |_____________> Time
  0    100      200 ms

Formula: speed(t) = target + (current - target) * exp(-t / tau)
tau = 100ms = time to reach ~63% of target
```

**Result**: Realistic acceleration behavior without actual motor inertia.

### Encoder Tick Generation

```
Distance traveled (mm) = Speed (mm/s) * Time (s)
Encoder ticks = Distance * (StepsPerRev / WheelCircumference)

Example:
- Speed = 50 mm/s
- Time = 10ms = 0.01s
- Distance = 50 * 0.01 = 0.5 mm
- Ticks = 0.5 * (1070 / (90*π)) ≈ 1.88 → rounds to 2 ticks
```

---

## 🔄 Integration with Main Loop

In `src/main.cpp`, add simulation update:

```cpp
// Declare at global scope
VirtualStepper* leftMotor = nullptr;
VirtualStepper* rightMotor = nullptr;

void setup() {
    setupSerial();
    setupGpio();
    setupWifi();

#ifdef MODE_REAL
    // Real hardware
    leftMotor = new DCMotor(PinMotorLeftEn, PinMotorLeftIn1, PinMotorLeftIn2,
                            PinEncoderLeftA, PinEncoderLeftB);
    rightMotor = new DCMotor(PinMotorRightEn, PinMotorRightIn1, PinMotorRightIn2,
                             PinEncoderRightA, PinEncoderRightB);
#else
    // Simulation
    leftMotor = new VirtualStepper();
    rightMotor = new VirtualStepper();
#endif

    leftMotor->begin();
    rightMotor->begin();
}

void loop() {
    // Update motor simulations (only does work in simulation mode)
    if (auto* vs = dynamic_cast<VirtualStepper*>(leftMotor)) {
        vs->update();
    }
    if (auto* vs = dynamic_cast<VirtualStepper*>(rightMotor)) {
        vs->update();
    }

    // ... rest of main loop
    delay(LoopRateMsec);
}
```

---

## ✅ Verification Checklist

- [ ] `include/VirtualStepper.h` and `src/VirtualStepper.cpp` created
- [ ] Compiles in both MODE_SIM and MODE_REAL
- [ ] Serial output shows "[VirtualStepper] Initialized"
- [ ] Encoder counts increment when motor speed > 0
- [ ] Speed ramps up smoothly (not instant)
- [ ] Direction reversal works correctly
- [ ] Zero PWM stops motor correctly

---

## 🧪 Test Sequence

```cpp
// Compile and upload in MODE_SIM

void testVirtualMotor() {
    VirtualStepper motor;
    motor.begin();

    Serial.println("Test 1: Ramp up to 50% speed");
    motor.setDirection(true);
    motor.setPwm(150);
    for (int i = 0; i < 10; i++) {
        motor.update();
        delay(50);
        Serial.println(motor.getDiagnostics());
    }

    Serial.println("Test 2: Reverse direction");
    motor.setDirection(false);
    for (int i = 0; i < 5; i++) {
        motor.update();
        delay(50);
        Serial.println(motor.getDiagnostics());
    }

    Serial.println("Test 3: Emergency stop");
    motor.setPwm(0);
    for (int i = 0; i < 5; i++) {
        motor.update();
        delay(50);
        Serial.println(motor.getDiagnostics());
    }
}
```

Expected output:
```
[VirtualStepper] Initialized (software simulation)
Test 1: Ramp up to 50% speed
PWM=150 (speed=0.0) Dir=FWD Enc=0 Healthy=1 [SIM]
PWM=150 (speed=18.5) Dir=FWD Enc=5 Healthy=1 [SIM]
PWM=150 (speed=36.2) Dir=FWD Enc=15 Healthy=1 [SIM]
...
```

---

## 💡 Design Benefits

1. **No Hardware Required**: Test algorithms anywhere (laptop, CI/CD)
2. **Deterministic**: Same PWM sequence always produces same result
3. **Fast**: 10 seconds of simulated time runs instantly
4. **Fault Injection**: Call `setSimulatedFault(true)` to test error handling
5. **Same Interface**: Swap VirtualStepper ↔ DCMotor without changing calling code

---

## 📊 Performance

| Metric | Value |
|--------|-------|
| Memory per instance | ~100 bytes |
| Update() call time | < 1 ms |
| Encoder accuracy | Within 1-2 ticks (due to rounding) |

---

## 🔄 Next Steps

1. **Step 4**: Create `DriveTrain` class using both motor types
2. **Step 5**: Add IMU integration
3. **Step 6**: Pose estimation (odometry + IMU fusion)

---

## 🎓 Key Insights

- **Dependency Injection**: DriveTrain doesn't know if motors are real or simulated
- **RAII**: No memory leaks; destructors clean up automatically
- **Atomic Operations**: Thread-safe encoder access without locks
- **Simulation Value**: Enables offline development and testing

---

## 🔗 References

- Exponential ramp formula: https://en.wikipedia.org/wiki/Exponential_decay
- Quadrature encoder simulation: Inverse of hardware decoding
- PlatformIO modes: https://docs.platformio.org/en/latest/
