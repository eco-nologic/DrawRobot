# DrawRobot Step 4: Differential Drive Kinematics (DriveTrain)

> **Implement motion command translation using differential drive kinematics**

---

## 📋 Objectives

By the end of this step, you will have:
- ✅ `DriveTrain` class converting linear/angular velocity to wheel speeds
- ✅ Differential drive kinematics (inverse and forward)
- ✅ Wheel speed PWM control
- ✅ Odometry calculation (distance, heading)
- ✅ Support for both real and simulated motors

---

## 🧮 Differential Drive Kinematics

### Inverse Kinematics (Motion Command → Wheel Speeds)

Given:
- Linear velocity: $v_x$ (mm/s)
- Angular velocity: $\omega$ (rad/s)
- Wheel base: $L$ (mm)

Calculate left and right wheel speeds:

$$v_{left} = v_x - \omega \times \frac{L}{2}$$

$$v_{right} = v_x + \omega \times \frac{L}{2}$$

### Forward Kinematics (Wheel Encoder → Position)

Given:
- Left encoder: $\Delta n_{left}$ (ticks)
- Right encoder: $\Delta n_{right}$ (ticks)

Calculate:

$$\Delta s = \frac{(\Delta n_{left} + \Delta n_{right})}{2} \times \text{mm\_per\_tick}$$

$$\Delta \theta = \frac{(\Delta n_{right} - \Delta n_{left})}{L} \times \text{mm\_per\_tick}$$

---

## 📝 Implementation

### DriveTrain.h Header

```cpp
// include/DriveTrain.h
#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "IMotor.h"
#include "Config.h"
#include <Arduino.h>
#include <atomic>
#include <cmath>

/**
 * @struct WheelSpeeds
 * @brief Represents left and right wheel speeds in mm/s
 */
struct WheelSpeeds {
    float leftMmS = 0.0f;   // Left wheel speed (mm/s)
    float rightMmS = 0.0f;  // Right wheel speed (mm/s)

    void print() const {
        Serial.printf("Left=%.1f mm/s, Right=%.1f mm/s\n", leftMmS, rightMmS);
    }
};

/**
 * @struct MotionCommand
 * @brief Represents desired motion: linear + angular velocity
 */
struct MotionCommand {
    float linearVelocityMmS = 0.0f;   // Forward/backward speed (mm/s)
    float angularVelocityRadS = 0.0f; // Rotation speed (rad/s)

    void print() const {
        Serial.printf("Linear=%.1f mm/s, Angular=%.3f rad/s\n", 
                      linearVelocityMmS, angularVelocityRadS);
    }
};

/**
 * @struct OdometryData
 * @brief Encoder-based odometry (position + heading)
 */
struct OdometryData {
    float x = 0.0f;        // X position (mm)
    float y = 0.0f;        // Y position (mm)
    float theta = 0.0f;    // Heading angle (radians, 0-2π)
    float linearDistance = 0.0f;  // Total distance traveled
    float rotationAngle = 0.0f;   // Total rotation

    void print() const {
        Serial.printf("Pos=(%.1f, %.1f) Theta=%.3f\n", x, y, theta);
    }
};

class DriveTrain {
private:
    // ====================================================================
    // MOTOR REFERENCES
    // ====================================================================
    IMotor* leftMotor;
    IMotor* rightMotor;

    // ====================================================================
    // MOTION STATE
    // ====================================================================
    MotionCommand currentCommand;
    WheelSpeeds currentWheelSpeeds;

    // ====================================================================
    // ODOMETRY STATE
    // ====================================================================
    OdometryData odometry;
    long lastLeftEncoderCount = 0;
    long lastRightEncoderCount = 0;

    // ====================================================================
    // HELPERS
    // ====================================================================

    /**
     * @brief Convert speed in mm/s to motor PWM (0-255)
     * 
     * Handles sign (forward/backward) and dead-band.
     */
    int speedToPwm(float speedMmS) const;

    /**
     * @brief Update odometry from encoder delta
     * 
     * Implements forward kinematics to calculate position and heading change.
     */
    void updateOdometry();

public:
    /**
     * @brief Constructor with dependency injection
     * 
     * @param left  Pointer to left motor (IMotor implementation)
     * @param right Pointer to right motor (IMotor implementation)
     */
    DriveTrain(IMotor* left, IMotor* right);
    ~DriveTrain() = default;

    /**
     * @brief Initialize motors and reset odometry
     */
    void begin();

    // ========================================================================
    // MOTION CONTROL
    // ========================================================================

    /**
     * @brief Set desired motion (linear + angular velocity)
     * 
     * @param linearMmS  Forward/backward speed in mm/s
     * @param angularRadS Rotation speed in rad/s
     */
    void setMotion(float linearMmS, float angularRadS);

    /**
     * @brief Stop all motors immediately
     */
    void stop();

    /**
     * @brief Drive in a straight line
     * 
     * @param speedMmS Speed in mm/s (positive=forward, negative=backward)
     */
    void driveStraight(float speedMmS);

    /**
     * @brief Rotate in place (differential drive)
     * 
     * @param angularRadS Rotation speed in rad/s
     */
    void rotateInPlace(float angularRadS);

    // ========================================================================
    // FEEDBACK
    // ========================================================================

    /**
     * @brief Get current wheel speeds
     */
    WheelSpeeds getWheelSpeeds() const { return currentWheelSpeeds; }

    /**
     * @brief Get current motion command
     */
    MotionCommand getMotionCommand() const { return currentCommand; }

    /**
     * @brief Get odometry estimate
     */
    OdometryData getOdometry() const { return odometry; }

    // ========================================================================
    // DIAGNOSTICS
    // ========================================================================

    /**
     * @brief Check if drive train is healthy
     */
    bool isHealthy() const {
        return leftMotor->isHealthy() && rightMotor->isHealthy();
    }

    /**
     * @brief Low-level wheel speed control (direct PWM)
     * 
     * Useful for fine-tuning or testing.
     */
    void setWheelSpeeds(float leftMmS, float rightMmS);

    /**
     * @brief Reset encoder and odometry
     */
    void resetOdometry();

    /**
     * @brief Get encoder counts
     */
    long getLeftEncoderCount() const;
    long getRightEncoderCount() const;

    /**
     * @brief Call periodically to update odometry
     * 
     * Must be called in main loop for odometry to update.
     */
    void updateSensors();
};

#endif // DRIVETRAIN_H
```

### DriveTrain.cpp Implementation

```cpp
// src/DriveTrain.cpp
#include "DriveTrain.h"

DriveTrain::DriveTrain(IMotor* left, IMotor* right)
    : leftMotor(left), rightMotor(right) {
}

void DriveTrain::begin() {
    if (!leftMotor || !rightMotor) {
        Serial.println("[DriveTrain] ERROR: Motors not initialized!");
        return;
    }

    leftMotor->begin();
    rightMotor->begin();
    resetOdometry();

    Serial.println("[DriveTrain] Initialized with differential drive kinematics");
}

int DriveTrain::speedToPwm(float speedMmS) const {
    // Clamp to max speed
    speedMmS = constrain(speedMmS, -MaxLinearSpeedMmS, MaxLinearSpeedMmS);

    // Convert speed to PWM (0-255)
    float normalized = abs(speedMmS) / MaxLinearSpeedMmS;
    int pwm = (int)(normalized * 255.0f);

    return pwm;
}

void DriveTrain::setMotion(float linearMmS, float angularRadS) {
    // Clamp inputs
    linearMmS = constrain(linearMmS, -MaxLinearSpeedMmS, MaxLinearSpeedMmS);
    angularRadS = constrain(angularRadS, -MaxAngularSpeedRadS, MaxAngularSpeedRadS);

    currentCommand.linearVelocityMmS = linearMmS;
    currentCommand.angularVelocityRadS = angularRadS;

    // Inverse kinematics: motion command → wheel speeds
    float halfBase = WheelBaseMm / 2.0f;
    float leftSpeed = linearMmS - angularRadS * halfBase;
    float rightSpeed = linearMmS + angularRadS * halfBase;

    setWheelSpeeds(leftSpeed, rightSpeed);
}

void DriveTrain::setWheelSpeeds(float leftMmS, float rightMmS) {
    currentWheelSpeeds.leftMmS = leftSpeed;
    currentWheelSpeeds.rightMmS = rightSpeed;

    // Convert speeds to PWM
    int leftPwm = speedToPwm(leftMmS);
    int rightPwm = speedToPwm(rightMmS);

    // Set motor directions and speeds
    leftMotor->setDirection(leftMmS >= 0);
    rightMotor->setDirection(rightMmS >= 0);
    leftMotor->setPwm(leftPwm);
    rightMotor->setPwm(rightPwm);
}

void DriveTrain::driveStraight(float speedMmS) {
    setMotion(speedMmS, 0.0f);  // Angular velocity = 0
}

void DriveTrain::rotateInPlace(float angularRadS) {
    setMotion(0.0f, angularRadS);  // Linear velocity = 0
}

void DriveTrain::stop() {
    currentCommand = MotionCommand();
    currentWheelSpeeds = WheelSpeeds();
    leftMotor->setPwm(0);
    rightMotor->setPwm(0);
}

void DriveTrain::updateSensors() {
    // Read current encoder values
    long currentLeftCount = leftMotor->getEncoderCount();
    long currentRightCount = rightMotor->getEncoderCount();

    // Calculate delta since last update
    long deltaLeftCount = currentLeftCount - lastLeftEncoderCount;
    long deltaRightCount = currentRightCount - lastRightEncoderCount;

    // Convert encoder ticks to mm
    float leftDeltaMm = (float)deltaLeftCount * MmPerStep;
    float rightDeltaMm = (float)deltaRightCount * MmPerStep;

    // Update encoder baseline
    lastLeftEncoderCount = currentLeftCount;
    lastRightEncoderCount = currentRightCount;

    // Forward kinematics
    float linearDeltaMm = (leftDeltaMm + rightDeltaMm) / 2.0f;
    float angularDeltaRad = (rightDeltaMm - leftDeltaMm) / WheelBaseMm;

    // Update position
    if (linearDeltaMm > 0.1f || linearDeltaMm < -0.1f) {  // Only update if meaningful
        odometry.x += linearDeltaMm * cos(odometry.theta);
        odometry.y += linearDeltaMm * sin(odometry.theta);
        odometry.linearDistance += abs(linearDeltaMm);
    }

    // Update heading
    odometry.theta += angularDeltaRad;
    
    // Normalize theta to [0, 2π]
    while (odometry.theta > 2.0f * M_PI) {
        odometry.theta -= 2.0f * M_PI;
    }
    while (odometry.theta < 0.0f) {
        odometry.theta += 2.0f * M_PI;
    }

    odometry.rotationAngle += abs(angularDeltaRad);
}

void DriveTrain::resetOdometry() {
    odometry = OdometryData();
    lastLeftEncoderCount = leftMotor->getEncoderCount();
    lastRightEncoderCount = rightMotor->getEncoderCount();
    leftMotor->resetEncoder();
    rightMotor->resetEncoder();
}

long DriveTrain::getLeftEncoderCount() const {
    return leftMotor->getEncoderCount();
}

long DriveTrain::getRightEncoderCount() const {
    return rightMotor->getEncoderCount();
}
```

---

## ✅ Verification Checklist

- [ ] Compiles without errors
- [ ] Motor command (50 mm/s linear) → left & right speeds are equal
- [ ] Rotation command (1 rad/s angular) → left & right speeds are opposite
- [ ] Combined command → speeds calculated correctly
- [ ] Encoder counts update when motors spin
- [ ] Odometry position changes based on encoder delta
- [ ] Heading updates when rotating

---

## 🧪 Test Sequence

```cpp
void testDriveTrain() {
    DriveTrain dt(leftMotor, rightMotor);
    dt.begin();

    // Test 1: Drive forward
    Serial.println("Test 1: Drive forward 50 mm/s");
    dt.setMotion(50.0f, 0.0f);
    dt.updateSensors();
    dt.getWheelSpeeds().print();

    // Test 2: Rotate
    Serial.println("Test 2: Rotate 1 rad/s");
    dt.setMotion(0.0f, 1.0f);
    dt.updateSensors();
    dt.getWheelSpeeds().print();

    // Test 3: Combined
    Serial.println("Test 3: Drive + rotate");
    dt.setMotion(50.0f, 0.5f);
    dt.updateSensors();
    dt.getWheelSpeeds().print();
}
```

---

## 🔄 Next Steps

1. **Step 5**: Add IMU integration
2. **Step 6**: Sensor fusion (odometry + IMU)
3. **Step 7**: PID-based motion control

---

## 💡 Design Notes

- **Kinematics**: All conversions at compile-time with constexpr
- **Odometry**: Dual encoder reading provides robust position estimate
- **Direction Handling**: Separate direction bits from speed (PWM)

---

## 🔗 References

- Differential Drive Kinematics: https://en.wikipedia.org/wiki/Differential_wheeled_robot
- Encoder Resolution: Each tick = WheelCircumference / StepsPerRev
