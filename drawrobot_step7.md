# DrawRobot Step 7: Motion Control (PID)

> **Implement PID-based motion controller for accurate waypoint following**

---

## 📋 Objectives

By the end of this step, you will have:
- ✅ `MotionController` with independent linear and angular PID loops
- ✅ Waypoint queue for path execution
- ✅ Error correction based on pose feedback
- ✅ Adaptive speed and rotation control
- ✅ Real-time diagnostics

---

## 📝 Key Implementation

### MotionController.h (Condensed)

```cpp
// include/MotionController.h
#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include "PoseEstimator.h"
#include "DriveTrain.h"
#include <Arduino.h>
#include <queue>

/**
 * @struct Waypoint
 * @brief Target position with arrival tolerance
 */
struct Waypoint {
    float x = 0.0f, y = 0.0f;           // Target position (mm)
    float targetHeading = 0.0f;         // Desired heading (rad)
    float linearSpeedMmS = 100.0f;      // Cruising speed
    float arrivalTolerance = 20.0f;     // Position tolerance (mm)

    Waypoint() = default;
    Waypoint(float x_, float y_, float speed = 100.0f)
        : x(x_), y(y_), linearSpeedMmS(speed) {}
};

class MotionController {
private:
    DriveTrain* driveTrain;
    PoseEstimator* poseEstimator;

    // PID parameters (from Config.h)
    static constexpr float Kp_lin = PidLinearKp;
    static constexpr float Ki_lin = PidLinearKi;
    static constexpr float Kd_lin = PidLinearKd;

    static constexpr float Kp_ang = PidAngularKp;
    static constexpr float Ki_ang = PidAngularKi;
    static constexpr float Kd_ang = PidAngularKd;

    // Waypoint queue
    std::queue<Waypoint> waypointQueue;
    Waypoint currentWaypoint;
    bool isFollowingPath = false;

    // PID state
    float errorLinearIntegral = 0.0f;
    float errorAngularIntegral = 0.0f;
    float lastErrorLinear = 0.0f;
    float lastErrorAngular = 0.0f;

public:
    MotionController(DriveTrain* dt, PoseEstimator* pe)
        : driveTrain(dt), poseEstimator(pe) {}

    void begin() {
        Serial.println("[MotionController] PID motion control initialized");
    }

    /**
     * @brief Queue a waypoint to follow
     */
    void queueWaypoint(const Waypoint& wp) {
        waypointQueue.push(wp);
    }

    /**
     * @brief Start following queued waypoints
     */
    void startPathFollow() {
        if (!waypointQueue.empty()) {
            currentWaypoint = waypointQueue.front();
            waypointQueue.pop();
            isFollowingPath = true;
        }
    }

    /**
     * @brief Update motion control (call in main loop)
     */
    void update() {
        if (!isFollowingPath) {
            driveTrain->stop();
            return;
        }

        Pose current = poseEstimator->getPose();

        // Calculate distance and heading to waypoint
        float dx = currentWaypoint.x - current.x;
        float dy = currentWaypoint.y - current.y;
        float distance = sqrt(dx * dx + dy * dy);

        // Check arrival
        if (distance < currentWaypoint.arrivalTolerance) {
            if (!waypointQueue.empty()) {
                currentWaypoint = waypointQueue.front();
                waypointQueue.pop();
            } else {
                isFollowingPath = false;
                driveTrain->stop();
                Serial.println("[MotionController] Path complete");
                return;
            }
        }

        // Desired heading to waypoint
        float desiredHeading = atan2(dy, dx);
        float headingError = desiredHeading - current.theta;

        // Normalize heading error to [-π, π]
        while (headingError > M_PI) headingError -= 2.0f * M_PI;
        while (headingError < -M_PI) headingError += 2.0f * M_PI;

        // PID for linear motion
        float linearError = distance;
        errorLinearIntegral += linearError;
        float linearDerivative = linearError - lastErrorLinear;
        float linearOutput = Kp_lin * linearError + Ki_lin * errorLinearIntegral + Kd_lin * linearDerivative;
        lastErrorLinear = linearError;

        // PID for angular motion
        float angularError = headingError;
        errorAngularIntegral += angularError;
        float angularDerivative = angularError - lastErrorAngular;
        float angularOutput = Kp_ang * angularError + Ki_ang * errorAngularIntegral + Kd_ang * angularDerivative;
        lastErrorAngular = angularError;

        // Clamp outputs
        linearOutput = constrain(linearOutput, -MaxLinearSpeedMmS, MaxLinearSpeedMmS);
        angularOutput = constrain(angularOutput, -MaxAngularSpeedRadS, MaxAngularSpeedRadS);

        driveTrain->setMotion(linearOutput, angularOutput);
    }

    /**
     * @brief Stop current path
     */
    void stop() {
        isFollowingPath = false;
        driveTrain->stop();
        while (!waypointQueue.empty()) waypointQueue.pop();
    }

    bool isMoving() const { return isFollowingPath; }
};

#endif // MOTION_CONTROLLER_H
```

---

## ✅ Verification

- [ ] Compiles without errors
- [ ] Waypoint queueing works
- [ ] Robot reaches waypoint within tolerance
- [ ] Path following smooth (no jerky motions)

---

## 🔄 Next Steps

1. **Step 8**: Path planning (geometry, text rendering)
2. **Step 9**: Pen control
3. **Step 10**: Configuration manager

---

## 💡 Key Features

- **Independent PID Loops**: Linear and angular velocity controlled separately
- **Waypoint Queue**: Allows complex multi-segment paths
- **Heading Error Normalization**: Prevents 360° rotations
- **Integral Windup Protection**: Anti-windup needed for production

---

## 📊 Tuning Guidelines

| Parameter | Range | Effect |
|-----------|-------|--------|
| Kp | 0.1-1.0 | Proportional response (higher = faster) |
| Ki | 0.01-0.1 | Steady-state error elimination |
| Kd | 0.05-0.2 | Damping (prevents overshoot) |

Start with Ki=0 and Kd=0, tune Kp for acceptable speed, then add Ki for precision.
