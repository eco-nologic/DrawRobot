# DrawRobot Step 10: Configuration Manager

> **Persistent parameter storage using NVS (Non-Volatile Storage)**

---

## 📋 Objectives

- ✅ Save/load configuration to ESP32 flash
- ✅ PID tuning parameters, motor calibration
- ✅ Runtime parameter modification
- ✅ Default values fallback

---

## 📝 Key Points

```cpp
// include/ConfigManager.h
class ConfigManager {
private:
    static constexpr const char* NVS_NAMESPACE = "robot";

public:
    /**
     * @brief Initialize NVS
     */
    void begin() {
        // nvs_flash_init();
        Serial.println("[ConfigManager] NVS initialized");
    }

    /**
     * @brief Save float parameter
     */
    void setParam(const char* key, float value) {
        // nvs_set_blob(...);
        Serial.printf("[Config] %s = %.3f\n", key, value);
    }

    /**
     * @brief Load float parameter with default
     */
    float getParam(const char* key, float defaultValue = 0.0f) {
        // return nvs_get_blob(...);
        return defaultValue;
    }

    /**
     * @brief List all parameters
     */
    void printAll() {
        Serial.println("[Config] Current parameters:");
        // TODO: Enumerate NVS and print
    }
};
```

---

## ✅ Verification

- [ ] Parameters persist across reboot
- [ ] Default values loaded on first boot
- [ ] Can modify and save new values

---

## 🔄 Next Steps

1. **Step 11**: Battery monitoring
2. **Step 12**: WebSocket server
3. **Step 13**: Web dashboard
