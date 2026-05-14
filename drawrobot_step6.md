# DrawRobot Step 6: Pose Estimation (Sensor Fusion)

> **Fuse odometry + IMU data for robust position and heading estimation**

---

## 📋 Objectives

By the end of this step, you will have:
- ✅ `PoseEstimator` class fusing encoder odometry with IMU heading
- ✅ Ghost pose tracking (pure odometry for comparison)
- ✅ Robust heading estimate combining gyro + magnetometer
- ✅ Position uncertainty management
- ✅ Real-time pose updates in telemetry

---

## 📝 Implementation

### PoseEstimator.h Header

```cpp
// include/PoseEstimator.h
#ifndef POSE_ESTIMATOR_H
#define POSE_ESTIMATOR_H

#include "DriveTrain.h"
#include "Navigation.h"
#include <Arduino.h>
#include <cmath>

/**
 * @struct Pose
 * @brief Robot position (x, y) and heading (theta)
 */
struct Pose {
    float x = 0.0f;      // Position X (mm)
    float y = 0.0f;      // Position Y (mm)
    float theta = 0.0f;  // Heading (radians, 0-2π)

    void print(const char* label = "Pose") const {
        Serial.printf("%s: (%.1f, %.1f) θ=%.3f\n", label, x, y, theta);
    }
};

class PoseEstimator {
private:
    DriveTrain* driveTrain;
    Navigation* navigation;

    // Estimated pose (from sensor fusion)
    Pose estimatedPose;
    
    // Ghost pose (pure odometry, for comparison)
    Pose ghostPose;

    // Last update time
    unsigned long lastUpdateTime = 0;

    // Complementary filter gain (0-1)
    // Higher = trust IMU more, lower = trust odometry more
    static constexpr float ImuWeight = 0.3f;

public:
    PoseEstimator(DriveTrain* dt, Navigation* nav)
        : driveTrain(dt), navigation(nav) {
    }

    void begin() {
        driveTrain->resetOdometry();
        estimatedPose = Pose();
        ghostPose = Pose();
        lastUpdateTime = millis();
        Serial.println("[PoseEstimator] Initialized with sensor fusion");
    }

    /**
     * @brief Update pose estimate (call in main loop)
     */
    void update() {
        if (!driveTrain || !navigation) {
            return;
        }

        unsigned long now = millis();
        float deltaTimeS = (float)(now - lastUpdateTime) / 1000.0f;
        lastUpdateTime = now;

        // Get odometry and IMU data
        OdometryData odo = driveTrain->getOdometry();
        const NavigationData& nav = navigation->getNavData();

        // Update ghost pose (pure odometry)
        ghostPose.x = odo.x;
        ghostPose.y = odo.y;
        ghostPose.theta = odo.theta;

        // Sensor fusion: combine odometry heading with IMU heading
        // Complementary filter: estimated = (1-w)*odometry + w*IMU
        float imuHeading = nav.heading;
        float odoHeading = odo.theta;

        // Smooth heading transition (avoid jumps)
        estimatedPose.theta = (1.0f - ImuWeight) * odoHeading + ImuWeight * imuHeading;
        
        // Position from odometry (IMU doesn't provide position)
        estimatedPose.x = odo.x;
        estimatedPose.y = odo.y;
    }

    /**
     * @brief Get fused pose estimate
     */
    const Pose& getPose() const {
        return estimatedPose;
    }

    /**
     * @brief Get ghost pose (pure odometry)
     */
    const Pose& getGhostPose() const {
        return ghostPose;
    }

    /**
     * @brief Get IMU heading
     */
    float getImuHeading() const {
        return navigation->getNavData().heading;
    }

    /**
     * @brief Reset pose to origin
     */
    void resetPose() {
        estimatedPose = Pose();
        ghostPose = Pose();
        driveTrain->resetOdometry();
    }
};

#endif // POSE_ESTIMATOR_H
```

---

## ✅ Verification Checklist

- [ ] `PoseEstimator.h` created and compiles
- [ ] Pose updates smoothly as robot moves
- [ ] Ghost pose matches odometry
- [ ] Heading blends odometry and IMU
- [ ] Reset works correctly

---

## 🔄 Next Steps

1. **Step 7**: PID motion control using pose feedback
2. **Step 8**: Path planning and waypoint following
3. **Step 9**: Pen control integration

---

## 💡 Sensor Fusion Strategy

**Complementary Filter**: Best balance between speed and accuracy
- Odometry: Accurate short-term, drifts over time
- IMU: Noisy short-term, stable long-term
- Fusion: Combines strengths of both

**Formula**:
$$\text{Heading}_{\text{fused}} = (1 - w) \times \theta_{\text{odo}} + w \times \theta_{\text{IMU}}$$

where $w \in [0, 1]$ is the IMU weight.
