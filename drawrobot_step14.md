# DrawRobot Step 14: Integration & Full System Testing

> **Integrate all components, test end-to-end functionality, and debug**

---

## 📋 Objectives

- ✅ Complete main.cpp with all component initialization
- ✅ System health checks on startup
- ✅ Main control loop running all subsystems
- ✅ End-to-end motion and drawing verification
- ✅ Performance profiling and optimization
- ✅ Bluetooth debug bridge for diagnostics

---

## 📝 Complete main.cpp

```cpp
// src/main.cpp - COMPLETE INTEGRATION

#include <Arduino.h>
#include "Config.h"
#include "IMotor.h"
#include "DCMotor.h"
#include "VirtualStepper.h"
#include "DriveTrain.h"
#include "Navigation.h"
#include "PoseEstimator.h"
#include "MotionController.h"
#include "PathPlanner.h"
#include "ConfigManager.h"
#include "BatteryMonitor.h"
#include "CommsManager.h"

// ============================================================================
// GLOBAL COMPONENT INSTANCES
// ============================================================================

// Motors
IMotor* leftMotor = nullptr;
IMotor* rightMotor = nullptr;

// Motion control
DriveTrain* driveTrain = nullptr;
Navigation* navigation = nullptr;
PoseEstimator* poseEstimator = nullptr;
MotionController* motionController = nullptr;
PathPlanner* pathPlanner = nullptr;

// Accessories
ConfigManager* configManager = nullptr;
BatteryMonitor* batteryMonitor = nullptr;
CommsManager* commsManager = nullptr;

// ============================================================================
// TIMING & STATE
// ============================================================================

unsigned long lastStatusPrint = 0;
unsigned long lastDiagnostic = 0;
int loopCount = 0;

// ============================================================================
// INITIALIZATION
// ============================================================================

void initMotors() {
    Serial.println("[Setup] Initializing motors...");

#ifdef MODE_REAL
    leftMotor = new DCMotor(
        PinMotorLeftEn, PinMotorLeftIn1, PinMotorLeftIn2,
        PinEncoderLeftA, PinEncoderLeftB
    );
    rightMotor = new DCMotor(
        PinMotorRightEn, PinMotorRightIn1, PinMotorRightIn2,
        PinEncoderRightA, PinEncoderRightB
    );
    Serial.println("[Setup] Real hardware motors selected (MODE_REAL)");
#else
    leftMotor = new VirtualStepper();
    rightMotor = new VirtualStepper();
    Serial.println("[Setup] Virtual simulation motors selected (MODE_SIM)");
#endif
}

void initSensors() {
    Serial.println("[Setup] Initializing sensors...");
    navigation = new Navigation();
    navigation->begin();
}

void initControllers() {
    Serial.println("[Setup] Initializing motion controllers...");
    
    driveTrain = new DriveTrain(leftMotor, rightMotor);
    driveTrain->begin();

    poseEstimator = new PoseEstimator(driveTrain, navigation);
    poseEstimator->begin();

    motionController = new MotionController(driveTrain, poseEstimator);
    motionController->begin();

    pathPlanner = new PathPlanner(motionController);
}

void initAccessories() {
    Serial.println("[Setup] Initializing accessories...");
    
    configManager = new ConfigManager();
    configManager->begin();

    batteryMonitor = new BatteryMonitor();
    batteryMonitor->begin();
}

void initCommunications() {
    Serial.println("[Setup] Initializing communications...");
    commsManager = new CommsManager(poseEstimator, batteryMonitor);
    commsManager->begin();
}

void systemHealthCheck() {
    Serial.println("\n========== SYSTEM HEALTH CHECK ==========");
    
    // Check motors
    Serial.printf("Left Motor: %s\n", leftMotor->isHealthy() ? "OK" : "FAULT");
    Serial.printf("Right Motor: %s\n", rightMotor->isHealthy() ? "OK" : "FAULT");
    
    // Check sensors
    Serial.printf("Navigation: %s\n", navigation->isHealthy() ? "OK" : "FAULT");
    
    // Check drivetrain
    Serial.printf("DriveTrain: %s\n", driveTrain->isHealthy() ? "OK" : "FAULT");
    
    // Check battery
    Serial.printf("Battery: %.2f V (%d%%)\n", 
                  batteryMonitor->getVoltage(),
                  (int)batteryMonitor->getPercentage());
    
    Serial.println("========================================\n");
}

void setup() {
    // Serial
    Serial.begin(115200);
    delay(500);

    Serial.println("\n\n========================================");
    Serial.printf("DrawRobot Firmware v%s\n", FirmwareVersion);
    Serial.println("========================================\n");

#ifdef MODE_REAL
    Serial.println(">>> RUNNING IN REAL HARDWARE MODE <<<\n");
#else
    Serial.println(">>> RUNNING IN SIMULATION MODE <<<\n");
#endif

    // GPIO
    pinMode(PinLedUser1, OUTPUT);
    pinMode(PinLedUser2, OUTPUT);
    digitalWrite(PinLedUser1, LOW);
    digitalWrite(PinLedUser2, LOW);

    // Initialize all components
    initMotors();
    initSensors();
    initControllers();
    initAccessories();
    initCommunications();

    // Health check
    systemHealthCheck();

    Serial.println("[Setup] Initialization complete! Starting main loop...\n");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // ========================================================================
    // SENSOR UPDATES (CRITICAL)
    // ========================================================================
    
    if (auto* vs = dynamic_cast<VirtualStepper*>(leftMotor)) {
        vs->update();
    }
    if (auto* vs = dynamic_cast<VirtualStepper*>(rightMotor)) {
        vs->update();
    }

    navigation->update();
    driveTrain->updateSensors();

    // ========================================================================
    // STATE ESTIMATION
    // ========================================================================
    
    poseEstimator->update();

    // ========================================================================
    // MOTION CONTROL
    // ========================================================================
    
    motionController->update();

    // ========================================================================
    // COMMUNICATIONS (TELEMETRY)
    // ========================================================================
    
    commsManager->sendTelemetry();

    // ========================================================================
    // DIAGNOSTICS & STATUS (EVERY 1 SECOND)
    // ========================================================================
    
    if (millis() - lastStatusPrint > 1000) {
        Pose p = poseEstimator->getPose();
        Serial.printf("[Loop %d] Pose: (%.0f, %.0f) θ=%.2f°  Motors: %s/%s  Batt: %.1fV\n",
                      loopCount,
                      p.x, p.y, p.theta * 180 / M_PI,
                      leftMotor->getDiagnostics(),
                      rightMotor->getDiagnostics(),
                      batteryMonitor->getVoltage());
        
        lastStatusPrint = millis();
        loopCount++;
    }

    // ========================================================================
    // STATUS LED (BLINK PATTERN)
    // ========================================================================
    
    bool ledState = (millis() / 500) % 2;
    digitalWrite(PinLedUser1, ledState);
    digitalWrite(PinLedUser2, motionController->isMoving() ? HIGH : LOW);

    // ========================================================================
    // BATTERY CHECK (EVERY 5 SECONDS)
    // ========================================================================
    
    if (millis() - lastDiagnostic > 5000) {
        batteryMonitor->update();
        
        if (batteryMonitor->isLow()) {
            Serial.println("[ALERT] LOW BATTERY - STOPPING MOTION");
            motionController->stop();
        }
        
        lastDiagnostic = millis();
    }

    // ========================================================================
    // LOOP TIMING (TARGET: 10ms = 100 Hz)
    // ========================================================================
    
    delay(LoopRateMsec);
}
```

---

## ✅ Comprehensive Test Checklist

### Hardware Tests (MODE_REAL)
- [ ] Both motors respond to PWM commands
- [ ] Encoders count correctly (move wheel by hand, count increases)
- [ ] IMU detected and calibrates
- [ ] Battery voltage reads correctly
- [ ] Servo (pen) moves smoothly
- [ ] LEDs blink on/off

### Motion Tests
- [ ] `driveTrain->driveStraight(100)` → robot moves forward
- [ ] `driveTrain->rotateInPlace(1.0)` → robot spins
- [ ] Odometry updates as robot moves
- [ ] Heading from IMU matches rotation

### Drawing Tests
- [ ] Pen wheel visible and in contact with paper
- [ ] `pathPlanner->planCircle()` → draws circle with pen as it moves
- [ ] `pathPlanner->planTriangle()` → draws triangle
- [ ] Straight line test: robot moves forward 100mm → pen draws line
- [ ] Rotation test: robot rotates 360° → pen draws circle (radius ≈ 130mm)

### Communication Tests
- [ ] WiFi AP "RobotWifi" visible
- [ ] Can connect to 192.168.4.1
- [ ] Dashboard loads
- [ ] Real-time position updates
- [ ] Buttons trigger drawing

### Integration Tests
- [ ] Robot starts up cleanly
- [ ] Serial diagnostics output correct
- [ ] Motion, drawing, communications work simultaneously
- [ ] Battery low warning triggers
- [ ] System recovers from errors

---

## 🧪 Test Scenarios

### Scenario 1: Simple Motion (5 min)
```
1. Start robot
2. Send: driveStraight(100 mm/s) for 2 seconds
3. Send: rotateInPlace(0.5 rad/s) for 1 rotation
4. Verify: Robot position from odometry
5. Verify: Heading from IMU ≈ gyro
```

### Scenario 2: Drawing Shapes (10 min)
```
1. Place robot on large blank paper
2. Draw circle (radius 50mm, center 0,0)
3. Lift robot (pen automatically disengages)
4. Move to new location on paper
5. Draw triangle
6. Return home (pen remains disengaged during movement to avoid drawing)
```

### Scenario 3: Long-Duration Run (30 min)
```
1. Execute complex path with multiple shapes
2. Monitor: Battery voltage drops from 16.8V
3. Verify: Low battery warning at 6.6V
4. Verify: No system crashes or glitches
```

---

## 🔧 Performance Profiling

Use diagnostics to measure:

| Metric | Target | How to Measure |
|--------|--------|----------------|
| Loop Rate | 100 Hz | Count updates in 1 sec |
| Telemetry Latency | < 50ms | Measure WebSocket delay |
| Motion Accuracy | < 20mm | Compare planned vs actual position |
| Encoder Reliability | 100% | Check for missed ticks |
| IMU Update Rate | 50+ Hz | Check heading update frequency |

---

## 🚀 Optimization Tips

1. **Reduce Serial Output**: Disable during high-speed motion
2. **Use DMA for I2C**: Offload IMU reading
3. **Cache Calculations**: Pre-compute sin/cos tables for smooth curves
4. **Interrupt Priority**: IMU ISR > Main loop
5. **Memory**: Monitor stack usage, watch for heap fragmentation

---

## 🐛 Debugging Techniques

### Issue: Robot drifts to one side
- **Check**: Motor speeds equal? Encoder readings balanced?
- **Fix**: Calibrate motor PWM or wheel sizes

### Issue: Drawing inaccurate
- **Check**: Is odometry drifting? IMU drift?
- **Fix**: Reduce motion speed, recalibrate sensors

### Issue: Slow responses
- **Check**: Serial output blocking loop?
- **Fix**: Use non-blocking I/O, reduce diagnostic frequency

### Issue: WebSocket disconnects
- **Check**: WiFi signal? JSON parse errors?
- **Fix**: Add error handling, reconnect logic

---

## 📊 Performance Baseline

On a healthy system (real hardware):
- **Boot time**: 3-5 seconds
- **Loop frequency**: 100 ± 2 Hz
- **Telemetry latency**: 50-100 ms
- **Motion response**: < 200 ms
- **Drawing accuracy**: ± 15-20 mm

---

## ✨ Production Checklist

- [ ] All tests pass
- [ ] No compiler warnings
- [ ] Memory usage < 80% of available
- [ ] No crashes in 1-hour runtime test
- [ ] Battery monitoring working
- [ ] Emergency stop (PWM=0) functional
- [ ] Documentation complete

---

## 🔄 Next: Advanced Features (Future Steps)

- Path optimization (shortest path)
- Obstacle avoidance
- Real Kalman filter (vs. complementary)
- Multi-robot coordination
- Bluetooth debug bridge
- ML-based motion prediction
```