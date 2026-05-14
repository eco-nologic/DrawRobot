# DrawRobot Step 1: Motor Interface Design (IMotor)

> **Establish the abstract motor interface with inheritance pattern for hardware abstraction**

---

## 📋 Objectives

By the end of this step, you will have:
- ✅ Abstract `IMotor` base class with pure virtual methods
- ✅ Understanding of polymorphism and dependency injection
- ✅ Clear interface for motor implementations (DCMotor, VirtualStepper)
- ✅ Unified API for speed control and encoder feedback
- ✅ No compiler dependencies on hardware pins yet

---

## 🏗️ Design Pattern: Strategy Pattern

The **Strategy Pattern** allows different motor implementations to be used interchangeably:

```
Client Code (DriveTrain)
         |
         v
    IMotor (interface)
      /    \
     /      \
  DCMotor  VirtualStepper
  (Real)   (Simulation)
```

**Benefits**:
- ✅ Switch implementations at runtime (or compile time)
- ✅ Test without hardware (inject VirtualStepper)
- ✅ Easy to add new motor types (stepper motors, servo-based drives)
- ✅ No coupling between DriveTrain and specific motor implementation

---

## 📝 Implementation

### IMotor.h (Pure Abstract Interface)

```cpp
// include/IMotor.h
#ifndef IMOTOR_H
#define IMOTOR_H

#include <Arduino.h>
#include <cstdint>

/**
 * @class IMotor
 * @brief Abstract interface for motor control
 * 
 * Defines the contract that all motor implementations must fulfill.
 * Used by DriveTrain to control motors without knowing the specific implementation.
 */
class IMotor {
public:
    // Destructor must be virtual for proper cleanup of derived classes
    virtual ~IMotor() = default;

    // ========================================================================
    // INITIALIZATION
    // ========================================================================
    
    /**
     * @brief Initialize motor hardware (GPIO, PWM, interrupts)
     * 
     * Called once at startup. Must set up all resources the motor needs.
     * For VirtualStepper, this is a no-op. For DCMotor, this configures GPIO.
     */
    virtual void begin() = 0;

    // ========================================================================
    // SPEED CONTROL (PWM)
    // ========================================================================
    
    /**
     * @brief Set motor speed via PWM (0-255)
     * 
     * @param pwm   PWM value: 0 = stopped, 255 = max speed
     *              Values below MotorPwmDeadband are treated as stopped
     */
    virtual void setPwm(int pwm) = 0;

    /**
     * @brief Set motor direction
     * 
     * @param forward   true = forward, false = backward
     */
    virtual void setDirection(bool forward) = 0;

    // ========================================================================
    // ENCODER FEEDBACK
    // ========================================================================
    
    /**
     * @brief Get current encoder count (cumulative)
     * 
     * @return Long encoder value (increments when motor rotates forward)
     *         Can be positive or negative depending on direction
     */
    virtual long getEncoderCount() const = 0;

    /**
     * @brief Reset encoder count to zero
     * 
     * Useful for measuring distance traveled in a segment
     */
    virtual void resetEncoder() = 0;

    // ========================================================================
    // DIAGNOSTICS
    // ========================================================================
    
    /**
     * @brief Check if motor is operating normally
     * 
     * @return true if motor is healthy (encoder feedback present, no faults)
     *         false if encoder is stuck, shorted, or not responding
     */
    virtual bool isHealthy() const = 0;

    /**
     * @brief Get diagnostic string for debugging
     * 
     * @return String describing motor state (pwm, encoder, direction, health)
     *         Example: "PWM=150 Enc=1234 Fwd=1 Healthy=1"
     */
    virtual const char* getDiagnostics() const = 0;
};

#endif // IMOTOR_H
```

---

## 🎯 Design Principles Applied

### 1. **Pure Virtual Methods**
Each method is declared `= 0`, meaning derived classes **must** implement it.

```cpp
virtual void begin() = 0;  // Must be implemented by DCMotor, VirtualStepper
```

### 2. **Const-Correctness**
Methods that don't modify state are marked `const`:

```cpp
virtual long getEncoderCount() const = 0;     // Read-only
virtual bool isHealthy() const = 0;           // Read-only
virtual void setPwm(int pwm) = 0;             // Modifies state
```

**Benefits**: Prevents accidental modifications, enables const references.

### 3. **Virtual Destructor**
The destructor is virtual to ensure proper cleanup of derived classes:

```cpp
virtual ~IMotor() = default;
```

Without this, deleting a `DCMotor*` through an `IMotor*` pointer would leak resources.

### 4. **Clear Documentation**
Each method has:
- What it does
- Input parameters
- Return values
- When it's called
- Special cases

---

## 🔗 Integration with DriveTrain

In `DriveTrain.h`, motors are held as **pointers to the interface**:

```cpp
class DriveTrain {
private:
    IMotor* leftMotor;    // Pointer to any IMotor implementation
    IMotor* rightMotor;   // Could be DCMotor or VirtualStepper

public:
    // Dependency injection: caller provides motor implementations
    DriveTrain(IMotor* left, IMotor* right)
        : leftMotor(left), rightMotor(right) { }

    void begin() {
        if (leftMotor) leftMotor->begin();      // Calls DCMotor::begin() or VirtualStepper::begin()
        if (rightMotor) rightMotor->begin();
    }

    // ... rest of DriveTrain
};
```

**Advantage**: DriveTrain code never mentions `DCMotor` or `VirtualStepper`. It only knows `IMotor`.

---

## 📚 Usage Examples

### Scenario 1: Real Hardware
```cpp
// In main.cpp (MODE_REAL)
IMotor* leftMotor = new DCMotor(
    PinMotorLeftEn, PinMotorLeftIn1, PinMotorLeftIn2,
    PinEncoderLeftA, PinEncoderLeftB
);
IMotor* rightMotor = new DCMotor(
    PinMotorRightEn, PinMotorRightIn1, PinMotorRightIn2,
    PinEncoderRightA, PinEncoderRightB
);

DriveTrain drivetrain(leftMotor, rightMotor);
drivetrain.begin();
```

### Scenario 2: Simulation (No Hardware)
```cpp
// In main.cpp (MODE_SIM)
IMotor* leftMotor = new VirtualStepper();
IMotor* rightMotor = new VirtualStepper();

DriveTrain drivetrain(leftMotor, rightMotor);
drivetrain.begin();
```

**Both scenarios use identical DriveTrain code!**

---

## ✅ Verification Checklist

- [ ] `include/IMotor.h` file created
- [ ] Compiles without errors:
  ```bash
  platformio run -e sim
  ```
- [ ] No warnings about pure virtual functions
- [ ] Documentation clearly describes each method
- [ ] Method signatures use `const` appropriately
- [ ] Virtual destructor is present

---

## 🧠 Learning Points

**Polymorphism in C++**:
- Virtual methods allow calling derived-class code through base-class pointers
- Enables strategy pattern: swap implementations at runtime
- Compiler generates virtual method table (vtable) for dispatch

**RAII Principle**:
- Each implementation (DCMotor, VirtualStepper) owns resources in constructor
- Destructor releases resources (GPIO, interrupts, allocated memory)
- No manual cleanup needed if using smart pointers

---

## 🔄 Next Steps

1. **Step 2**: Implement `DCMotor.h` and `DCMotor.cpp` (real hardware)
2. **Step 3**: Implement `VirtualStepper.h` and `VirtualStepper.cpp` (simulation)
3. **Step 4**: Use both in `DriveTrain` class

---

## 💾 Memory Footprint

- `IMotor.h`: Interface only, no runtime memory
- Virtual method calls: ~2-4 CPU cycles overhead vs. direct calls (negligible)
- Derived classes add one `vptr` (8 bytes on 32-bit systems) per instance

**Conclusion**: Abstraction layer is essentially free in terms of performance.

---

## 🔗 References

- C++ Virtual Functions: https://en.cppreference.com/w/cpp/language/virtual
- Design Patterns (Gang of Four): Strategy Pattern
- RAII (Resource Acquisition Is Initialization): https://en.cppreference.com/w/cpp/language/raii
