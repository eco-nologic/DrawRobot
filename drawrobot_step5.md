# DrawRobot Step 5: IMU Sensor Integration

> **Integrate 9-axis IMU (accelerometer, gyroscope, magnetometer) with auto-detection**

---

## 📋 Objectives

By the end of this step, you will have:
- ✅ Auto-detection of MPU9250 or BNO055 IMU chips
- ✅ I2C communication for sensor reading
- ✅ Calibration procedures for accelerometer and magnetometer
- ✅ Heading estimation from gyroscope and magnetometer
- ✅ Acceleration data for dynamic motion detection

---

## 📝 Implementation

### Navigation.h Header (IMU Interface)

```cpp
// include/Navigation.h
#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "Config.h"
#include <Arduino.h>
#include <Wire.h>
#include <cmath>

/**
 * @struct ImuData
 * @brief Raw IMU readings (accel, gyro, magnet)
 */
struct ImuData {
    // Accelerometer (m/s^2)
    float accelX = 0.0f, accelY = 0.0f, accelZ = 0.0f;
    
    // Gyroscope (rad/s)
    float gyroX = 0.0f, gyroY = 0.0f, gyroZ = 0.0f;
    
    // Magnetometer (micro-Tesla)
    float magX = 0.0f, magY = 0.0f, magZ = 0.0f;
};

/**
 * @struct NavigationData
 * @brief High-level navigation state (heading, acceleration)
 */
struct NavigationData {
    float heading = 0.0f;        // Yaw angle (radians, 0-2π) from magnetometer
    float headingRate = 0.0f;    // Yaw rate (rad/s) from gyroscope
    float totalAccel = 0.0f;     // Total acceleration magnitude (m/s^2)
};

class Navigation {
private:
    // IMU type detection
    enum class ImuType {
        Unknown,
        MPU9250,
        BNO055
    };

    ImuType detectedImu = ImuType::Unknown;
    bool initialized = false;

    // Sensor calibration offsets
    float accelOffsetX = 0.0f, accelOffsetY = 0.0f, accelOffsetZ = 0.0f;
    float gyroOffsetX = 0.0f, gyroOffsetY = 0.0f, gyroOffsetZ = 0.0f;

    // Current sensor data
    ImuData imuRaw;
    NavigationData navData;

    // Helpers
    ImuType detectImu();
    void calibrateAccelerometer();
    void calibrateGyroscope();
    float calculateHeading(float magX, float magY);

public:
    Navigation() = default;
    ~Navigation() = default;

    /**
     * @brief Initialize I2C and detect IMU
     */
    void begin();

    /**
     * @brief Read IMU sensors (call in main loop)
     */
    void update();

    /**
     * @brief Get raw IMU data
     */
    const ImuData& getRawData() const { return imuRaw; }

    /**
     * @brief Get high-level navigation data
     */
    const NavigationData& getNavData() const { return navData; }

    /**
     * @brief Check if IMU is working
     */
    bool isHealthy() const { return initialized; }
};

#endif // NAVIGATION_H
```

### Navigation.cpp Implementation

```cpp
// src/Navigation.cpp
#include "Navigation.h"

Navigation::ImuType Navigation::detectImu() {
    Wire.begin(PinI2cSda, PinI2cScl, I2cFrequency);
    delay(100);

    // Try MPU9250
    Wire.beginTransmission(AddrMpu9250);
    if (Wire.endTransmission() == 0) {
        Serial.println("[Navigation] Detected MPU9250");
        return ImuType::MPU9250;
    }

    // Try BNO055
    Wire.beginTransmission(AddrBno055);
    if (Wire.endTransmission() == 0) {
        Serial.println("[Navigation] Detected BNO055");
        return ImuType::BNO055;
    }

    Serial.println("[Navigation] ERROR: No IMU detected!");
    return ImuType::Unknown;
}

void Navigation::calibrateAccelerometer() {
    Serial.println("[Navigation] Calibrating accelerometer... keep robot stationary for 2 seconds");
    
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    const int samples = 100;

    for (int i = 0; i < samples; i++) {
        update();
        sumX += imuRaw.accelX;
        sumY += imuRaw.accelY;
        sumZ += imuRaw.accelZ;
        delay(20);
    }

    accelOffsetX = sumX / samples;
    accelOffsetY = sumY / samples;
    accelOffsetZ = sumZ / samples - 9.81f;  // Remove gravity

    Serial.printf("[Navigation] Accel offsets: X=%.3f Y=%.3f Z=%.3f\n",
                  accelOffsetX, accelOffsetY, accelOffsetZ);
}

void Navigation::calibrateGyroscope() {
    Serial.println("[Navigation] Calibrating gyroscope... keep robot stationary for 2 seconds");
    
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    const int samples = 100;

    for (int i = 0; i < samples; i++) {
        update();
        sumX += imuRaw.gyroX;
        sumY += imuRaw.gyroY;
        sumZ += imuRaw.gyroZ;
        delay(20);
    }

    gyroOffsetX = sumX / samples;
    gyroOffsetY = sumY / samples;
    gyroOffsetZ = sumZ / samples;

    Serial.printf("[Navigation] Gyro offsets: X=%.3f Y=%.3f Z=%.3f\n",
                  gyroOffsetX, gyroOffsetY, gyroOffsetZ);
}

float Navigation::calculateHeading(float magX, float magY) {
    // Calculate heading from magnetometer X and Y
    // atan2 returns angle in [-π, π], convert to [0, 2π]
    float heading = atan2(magY, magX);
    if (heading < 0) {
        heading += 2.0f * M_PI;
    }
    return heading;
}

void Navigation::begin() {
    detectedImu = detectImu();
    
    if (detectedImu == ImuType::Unknown) {
        Serial.println("[Navigation] Initialization failed!");
        initialized = false;
        return;
    }

    calibrateAccelerometer();
    calibrateGyroscope();

    initialized = true;
    Serial.println("[Navigation] IMU initialized and calibrated");
}

void Navigation::update() {
    if (!initialized) {
        return;
    }

    // TODO: Implement sensor-specific read methods
    // For MPU9250: read via I2C registers
    // For BNO055: use BNO055 API
    
    // Apply calibration offsets
    imuRaw.accelX -= accelOffsetX;
    imuRaw.accelY -= accelOffsetY;
    imuRaw.accelZ -= accelOffsetZ;
    imuRaw.gyroX -= gyroOffsetX;
    imuRaw.gyroY -= gyroOffsetY;
    imuRaw.gyroZ -= gyroOffsetZ;

    // Calculate heading
    navData.heading = calculateHeading(imuRaw.magX, imuRaw.magY);
    navData.headingRate = imuRaw.gyroZ;  // Z-axis gyro for yaw

    // Calculate total acceleration
    navData.totalAccel = sqrt(imuRaw.accelX * imuRaw.accelX +
                             imuRaw.accelY * imuRaw.accelY +
                             imuRaw.accelZ * imuRaw.accelZ);
}
```

---

## ✅ Verification Checklist

- [ ] `Navigation.h` and `Navigation.cpp` created
- [ ] Compiles without errors
- [ ] I2C initialization succeeds
- [ ] IMU detected (MPU9250 or BNO055)
- [ ] Sensor calibration completes
- [ ] Heading updates as robot rotates
- [ ] Acceleration magnitude responds to motion

---

## 🔄 Next Steps

1. **Step 6**: Sensor fusion (odometry + IMU)
2. **Step 7**: PID motion control
3. **Step 8**: Path planning

---

## 💡 Key Points

- Auto-detection simplifies hardware deployment
- Calibration removes bias errors
- Heading from magnetometer + gyroscope provides robust yaw
