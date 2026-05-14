# DrawRobot: Comprehensive C++ Implementation Plan

> **A precision differential-drive drawing robot with IMU navigation and web-based control**
>
> Built with professional C++ architecture: interfaces, inheritance, RAII, const-correctness, and memory optimization.

---

## 🎯 Project Vision

Build a fully autonomous pen-drawing robot that:
- 🎮 Controls two differential-drive motors with encoder feedback
- 🧭 Integrates 9-axis IMU for precise heading and acceleration sensing
- 📍 Fuses odometry + IMU for robust position tracking (odometry + gyro + accelerometer)
- ✏️ Draws geometric shapes (circles, triangles, rectangles, spirals)
- 📝 Writes text with font rendering and smooth Bezier curves
- 🌐 Provides real-time web dashboard with telemetry streaming
- 🔋 Monitors battery health and motor performance
- 🐛 Supports Bluetooth debugging bridge for diagnostics

---

## 📊 System Architecture Overview

### Layered Component Model
# DrawRobot Step 15: Data Analysis & Validation

> **Automate the generation of validation graphics and sensor calibration**

---

## 📋 Objectives

- ✅ Export Bluetooth/WebSocket telemetry to CSV
- ✅ Generate $l_{mes}$ vs $l_{th}$ and $\theta_{mes}$ vs $\theta_{th}$ graphs for the report
- ✅ Compute Hard-Iron offsets for Magnetometer calibration
- ✅ Detect EMI (Electromagnetic Interference) and vibration issues
- ✅ Fulfill the "Acquisition temporelle" requirement (p.17 of travail.pdf)

---

## 📝 Analysis Logic

The analysis script (typically in Python using `pandas` and `matplotlib`) performs three critical functions for the defense:

### 1. Spatial Validation (Requirement ET1.2 / ET2.3)
By plotting the $(x, y)$ coordinates from the `PoseEstimator`, we can visually confirm if the stairs are orthogonal and if the circle closes within 5mm.

### 2. Magnetometer Calibration (Requirement ET3.1)
To find the North with $\pm10^\circ$ accuracy, we must remove the robot's own magnetic field. 
- **Method**: Rotate the robot 360°. The script finds the center of the magnetic circle.
- **Correction**: $\text{Offset} = \frac{\max + \min}{2}$. These values are then saved in `ConfigManager`.

### 3. Stability Check
The script analyzes heading drift over time to justify the choice of your sensor fusion parameters (Mahony/Complementary filter).

---

## 🛡️ Defense Arguments (DEFENSE/ANSWER)

// DEFENSE: "Comment avez-vous validé la précision de 1cm demandée ?"
// ANSWER: "Nous avons développé un script Python qui récupère les logs du robot via Bluetooth. Il compare le tracé réel (odométrie fusionnée) aux dimensions théoriques et génère les graphiques de performance du rapport."

// DEFENSE: "Comment le magnétomètre est-il calibré pour trouver le Nord ?"
// ANSWER: "Le robot effectue une rotation sur lui-même pour mapper les perturbations magnétiques locales (Hard-Iron). Le script d'analyse calcule les offsets nécessaires pour recentrer les mesures et garantir un CAP fiable."

---

## ✅ Verification

- [ ] Robot streams data to `log_telemetry.py`
- [ ] CSV file is generated with correct timestamps
- [ ] `analyze_robot.py` generates the 4 requested charts (XY, Heading, Distance, Battery)
- [ ] Final closure error is calculated and displayed in the console
```

```
┌─────────────────────────────────────────────────────────────┐
│                    Web Interface Layer                      │
│              (Dashboard, Controls, Visualization)           │
└──────────────────────┬──────────────────────────────────────┘
                       │ (JSON over WebSocket)
┌──────────────────────┴──────────────────────────────────────┐
│                   Communication Layer                       │
│        (CommsManager, WebSocket Server, JSON Telemetry)     │
└──────────────────────┬──────────────────────────────────────┘
                       │
┌──────────────────────┴──────────────────────────────────────┐
│              Command Planning & Execution Layer             │
│  (PathPlanner, MotionController, VirtualStepper Simulation) │
└──────────────────────┬──────────────────────────────────────┘
                       │
┌──────────────────────┴──────────────────────────────────────┐
│              Sensor Fusion & State Layer                    │
│     (PoseEstimator, Navigation, ConfigManager, Battery)     │
└──────────────────────┬──────────────────────────────────────┘
                       │
┌──────────────────────┴──────────────────────────────────────┐
│              Motor & Control Layer                          │
│       (DriveTrain, Motor Control, Encoder Reading)          │
└──────────────────────┬──────────────────────────────────────┘
                       │
┌──────────────────────┴──────────────────────────────────────┐
│              Hardware Abstraction Layer                     │
│     (IMotor Interface, DCMotor, VirtualStepper Drivers)     │
└──────────────────────┬──────────────────────────────────────┘
                       │
                    Hardware
                   (Motors, Encoders, IMU, WiFi)
```

---

## 📋 Implementation Steps (15 phases)

Each step is **self-contained** but integrates with others via well-defined interfaces.

| # | File | Component | Focus | Dependencies | Complexity |
|---|------|-----------|-------|--------------|-----------|
| 0 | [drawrobot_step0.md](drawrobot_step0.md) | **Foundation** | Project structure, hardware config, CMake/PlatformIO setup | None | ⭐ |
| 1 | [drawrobot_step1.md](drawrobot_step1.md) | **Motor Interface** | Abstract IMotor interface, inheritance pattern | Step 0 | ⭐ |
| 2 | [drawrobot_step2.md](drawrobot_step2.md) | **DC Motor Driver** | DCMotor class, PWM, encoder reading, fault detection | Step 1 | ⭐⭐ |
| 3 | [drawrobot_step3.md](drawrobot_step3.md) | **Virtual Motor** | VirtualStepper for simulation, no hardware required | Step 1 | ⭐ |
| 4 | [drawrobot_step4.md](drawrobot_step4.md) | **Drive Train** | Differential drive kinematics, wheel speed control | Step 2, 3 | ⭐⭐ |
| 5 | [drawrobot_step5.md](drawrobot_step5.md) | **IMU Integration** | Auto-detect MPU9250/BNO055, calibration, sensor fusion | Step 0 | ⭐⭐ |
| 6 | [drawrobot_step6.md](drawrobot_step6.md) | **Pose Estimation** | Odometry + IMU fusion, ghost pose, state tracking | Step 4, 5 | ⭐⭐⭐ |
| 7 | [drawrobot_step7.md](drawrobot_step7.md) | **Motion Control** | PID loops, waypoint following, trajectory execution | Step 6 | ⭐⭐⭐ |
| 8 | [drawrobot_step8.md](drawrobot_step8.md) | **Path Planning** | Geometry (circles, lines, spirals), text rendering, Bezier | Step 7 | ⭐⭐⭐ |
| 9 | [drawrobot_step9.md](drawrobot_step9.md) | **Pen Wheel** | Passive third wheel mechanics, drawing, wear monitoring | Step 7 | ⭐⭐ |
| 10 | [drawrobot_step10.md](drawrobot_step10.md) | **Configuration** | NVS storage, parameter persistence, dynamic tuning | Step 0 | ⭐⭐ |
| 11 | [drawrobot_step11.md](drawrobot_step11.md) | **Battery Monitor** | ADC reading, voltage monitoring, health diagnostics | Step 0 | ⭐ |
| 12 | [drawrobot_step12.md](drawrobot_step12.md) | **WebSocket Server** | JSON telemetry, WiFi AP setup, real-time streaming | Step 0 | ⭐⭐⭐ |
| 13 | [drawrobot_step13.md](drawrobot_step13.md) | **Web Dashboard** | HTML/CSS/JS frontend, controls, map visualization | Step 12 | ⭐⭐⭐⭐ |
| 14 | [drawrobot_step14.md](drawrobot_step14.md) | **Integration & Debug** | Full system test, Bluetooth debug bridge, tuning | All steps | ⭐⭐⭐⭐ |
| 15 | [drawrobot_step15.md](drawrobot_step15.md) | **Data Analysis & Validation** | Automated reporting, IMU/Mag calibration, Python charting | Step 14 | ⭐⭐⭐ |

---

## 🏗️ C++ Architecture Principles

### 1. **Interface-Based Design (IMotor Pattern)**
```cpp
class IMotor {
    virtual void begin() = 0;
    virtual void setPwm(int pwm) = 0;
    virtual void setDirection(bool forward) = 0;
    virtual long getEncoderCount() = 0;
    virtual ~IMotor() = default;
};
```
**Benefits**: Swap implementations (Real ↔ Virtual), unit testing, dependency injection

### 2. **RAII (Resource Acquisition Is Initialization)**
- Constructors initialize resources (GPIO, I2C, WiFi)
- Destructors clean up automatically
- No memory leaks from exceptions

### 3. **Const-Correctness**
```cpp
float getPose() const { return pose; }          // Read-only access
void updatePose(float newPose) { ... }          // Modifying access
const WheelSpeeds& getWheelSpeeds() const { ... } // No copies
```

### 4. **Dependency Injection**
```cpp
DriveTrain::DriveTrain(IMotor* left, IMotor* right)
    : leftMotor(left), rightMotor(right) { }
```
Allows flexible initialization without tight coupling

### 5. **Inheritance Hierarchy**
```
IMotor (abstract interface)
  ├─ DCMotor (hardware with encoder)
  └─ VirtualStepper (software simulation)
```

### 6. **Performance Optimization**
- Use `const&` for pass-by-reference (avoid copies)
- Pre-allocate buffers in constructors
- Use `constexpr` for compile-time constants
- Inline small functions in headers
- Cache expensive calculations (sin/cos lookups)

### 7. **Modularité et Simplicité (Tips)**
- Division des fonctions complexes en sous-fonctions simples et descriptives.
- Facilite la lecture du code et permet d'expliquer chaque bloc de logique indépendamment lors de la soutenance.
- Exemple: La boucle `loop()` appelle `runControlCycle()`, qui lui-même délègue aux modules.

### 8. **Préparation à la Soutenance (Mandatory)**
Il est obligatoire d'inclure des commentaires structurés pour anticiper les questions du jury :
- `DEFENSE:` Suivi de la question technique probable (en français).
- `ANSWER:` Suivi de la justification technique détaillée (en français).

*Exemple :*
`// DEFENSE: "Pourquoi utiliser un PID ?" / ANSWER: "Pour corriger l'erreur statique et garantir une précision de tracé < 1cm."`

---

## 🔧 Hardware Quick Reference

### Physical Robot
| Parameter | Value |
|-----------|-------|
| Wheel Diameter | 90 mm |
| Wheel Base | 83 mm |
| Pen Offset | 130 mm (distance from axle to pen) |
| Encoder Steps/Rev | 1070 |
| Max Linear Speed | 120 mm/s |
| Max Angular Speed | 2.9 rad/s |

### GPIO Pinout (from pin.pdf)
| Component | Pin | Type |
|-----------|-----|------|
| Left Motor EN | GPIO 4 | PWM |
| Left Motor IN1 | GPIO 17 | Out |
| Left Motor IN2 | GPIO 16 | Out |
| Right Motor EN | GPIO 23 | PWM |
| Right Motor IN1 | GPIO 19 | Out |
| Right Motor IN2 | GPIO 18 | Out |
| Left Encoder A | GPIO 33 | In |
| Left Encoder B | GPIO 32 | In |
| Right Encoder A | GPIO 27 | In |
| Right Encoder B | GPIO 14 | In |
| I2C SDA | GPIO 21 | I2C |
| I2C SCL | GPIO 22 | I2C |
| Battery ADC | GPIO 0 | ADC |

**Note**: Pen is a passive third wheel (no servo control). Drawing occurs automatically as robot moves.

### WiFi Configuration
```cpp
SSID:      "RobotWifi"
Password:  "penbot123"
IP:        192.168.4.1
Port:      80 (WebSocket)
```

---

## 🚀 Development Workflow

### For Hardware Testing
```bash
# Define MODE_REAL in platformio.ini to use actual motors/sensors
[env:real_robot]
build_flags = -DMODE_REAL

# For simulation (safe testing without hardware):
[env:simulation]
# Default, uses VirtualStepper
```

### Build & Upload
```bash
platformio run -e real_robot --target upload
platformio monitor -b 115200
```

### Web Dashboard
```
1. Connect to "RobotWifi" WiFi
2. Open http://192.168.4.1 in browser
3. Use controls to test motion, drawing, and diagnostics
```

---

## 📈 Key Design Patterns Used

| Pattern | Used For | Example |
|---------|----------|---------|
| **Strategy** | Motor implementations | IMotor → DCMotor, VirtualStepper |
| **Dependency Injection** | Component coupling | DriveTrain(IMotor*, IMotor*) |
| **State Machine** | Motion state | IDLE → MOVING → DRAWING → STOPPED |
| **Observer** | Telemetry updates | CommsManager observes all modules |
| **Singleton** | Global state | ConfigManager, Navigation |
| **Template** (if needed) | Generic collections | Path waypoints, sensor buffers |

---

## ✅ Verification Checklist by Step

Each step includes:
- ✔️ Unit test code snippets
- ✔️ Integration checkpoints
- ✔️ Hardware validation procedures
- ✔️ Performance targets (timing, memory)

---

## 🔄 Iterative Development Approach

1. **Build foundation** (Steps 0-3): Config, motor interface, implementations
2. **Test hardware** (Steps 2-4): Verify motors work independently
3. **Add sensors** (Steps 5-6): IMU integration, position tracking
4. **Implement control** (Steps 7-9): Motion commands, pen control
5. **Add communications** (Steps 12-13): Web interface
6. **Full integration** (Step 14): End-to-end testing, tuning

**Each step is validated before moving to the next**, ensuring stability.

---

## 📚 Reference Materials

- **travail.pdf**: Project specifications and requirements
- **pin.pdf**: Hardware GPIO pinout and I2C addresses
- **example/**: Previous GiRobot project (architecture guide only)

---

## 🎓 Learning Outcomes

By completing this project, you'll master:
- ✅ Modern C++ (C++17) with interfaces and inheritance
- ✅ Embedded systems: GPIO, PWM, I2C, ADC, SPI
- ✅ Sensor fusion algorithms (Kalman-like odometry + IMU)
- ✅ Control systems: PID loops, trajectory planning
- ✅ Real-time systems: interrupt handling, timing
- ✅ Network programming: WebSocket, WiFi AP
- ✅ Web development: frontend dashboard, JSON API
- ✅ Hardware debugging and optimization

---

## 🛠️ Tools & Platform

- **MCU**: ESP32 (dual-core Xtensa with WiFi)
- **IDE**: PlatformIO (VSCode extension)
- **Language**: C++17
- **Frameworks**: Arduino, ESP-IDF (native)
- **Web**: HTML5 + CSS3 + JavaScript (no build tools required)

---

**Next**: Start with [drawrobot_step0.md](drawrobot_step0.md) - Foundation & Project Setup
