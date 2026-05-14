# DrawRobot Step 11: Battery Monitoring

> **Monitor battery voltage and detect low-power conditions**

---

## 📋 Objectives

- ✅ Read battery voltage via ADC (GPIO 0)
- ✅ Voltage-to-percentage conversion
- ✅ Low battery warning and shutdown
- ✅ Telemetry streaming to dashboard

---

## 📝 Implementation

```cpp
// include/BatteryMonitor.h
#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include "Config.h"
#include <Arduino.h>

class BatteryMonitor {
private:
    float currentVoltage = 16.8f;  // Start at full
    float previousVoltage = 16.8f;
    bool lowBatteryWarning = false;

public:
    void begin() {
        analogSetAttenuation(ADC_11db);  // 0-3.6V range
        pinMode(PinBatteryAdc, INPUT);
        Serial.println("[Battery] ADC monitoring enabled");
    }

    /**
     * @brief Read battery voltage (call periodically)
     */
    void update() {
        // Read ADC value (0-4095 for 0-3.3V)
        int adcRaw = analogRead(PinBatteryAdc);
        float adcVoltage = (float)adcRaw / 4095.0f * 3.3f;
        
        // Apply voltage divider ratio
        currentVoltage = adcVoltage * BatteryDividerRatio;

        // Low battery check
        if (currentVoltage < BatteryLowVoltage && !lowBatteryWarning) {
            lowBatteryWarning = true;
            Serial.printf("[Battery] LOW BATTERY WARNING: %.2f V\n", currentVoltage);
        }

        previousVoltage = currentVoltage;
    }

    /**
     * @brief Get current voltage
     */
    float getVoltage() const { return currentVoltage; }

    /**
     * @brief Get battery percentage (0-100)
     */
    float getPercentage() const {
        // Linear: 6.6V = 0%, 16.8V = 100% (4S LiPo)
        float pct = (currentVoltage - BatteryLowVoltage) / 
                    (BatteryFullVoltage - BatteryLowVoltage) * 100.0f;
        return constrain(pct, 0.0f, 100.0f);
    }

    /**
     * @brief Check if battery is low
     */
    bool isLow() const { return lowBatteryWarning; }
};

#endif // BATTERY_MONITOR_H
```

---

## ✅ Verification

- [ ] ADC reads stable voltage
- [ ] Warning triggers below 6.6V
- [ ] Percentage calculation reasonable
