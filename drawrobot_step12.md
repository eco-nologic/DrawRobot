# DrawRobot Step 12: WebSocket Server & Telemetry

> **Stream telemetry and receive commands over WiFi WebSocket**

---

## 📋 Objectives

- ✅ WiFi Access Point setup
- ✅ WebSocket server on port 80
- ✅ JSON telemetry streaming (pose, sensors, battery)
- ✅ Command reception (motion, drawing, config)
- ✅ Real-time performance metrics

---

## 📝 Key Implementation

```cpp
// include/CommsManager.h
#ifndef COMMS_MANAGER_H
#define COMMS_MANAGER_H

#include <WiFi.h>
#include <AsyncWebServer.h>
#include <AsyncWebSocket.h>
#include <ArduinoJson.h>
#include "Config.h"
#include "PoseEstimator.h"
#include "BatteryMonitor.h"
#include "MotionController.h"
#include "PathPlanner.h"

class CommsManager {
private:
    AsyncWebServer server{80};
    AsyncWebSocket ws{"/ws"};
    PoseEstimator* poseEstimator;
    BatteryMonitor* batteryMonitor;
    MotionController* motionController;
    PathPlanner* pathPlanner;
    unsigned long lastTelemetryTime = 0;

public:
    CommsManager(PoseEstimator* pe, BatteryMonitor* bm, 
                 MotionController* mc, PathPlanner* pp)
        : poseEstimator(pe), batteryMonitor(bm), 
          motionController(mc), pathPlanner(pp) {}

    void begin() {
        // Start WiFi AP
        WiFi.mode(WIFI_AP);
        WiFi.softAP(WifiSsid, WifiPassword);
        WiFi.softAPConfig(
            IPAddress(192, 168, 4, 1),
            IPAddress(192, 168, 4, 1),
            IPAddress(255, 255, 255, 0)
        );

        // Setup WebSocket
        ws.onEvent([this](AsyncWebSocket* server, AsyncWebSocketClient* client,
                         AwsEventType type, void* arg, uint8_t* data, size_t len) {
            if (type == WS_EVT_CONNECT) {
                Serial.printf("[Comms] Client %u connected\n", client->id());
            } else if (type == WS_EVT_DISCONNECT) {
                Serial.printf("[Comms] Client %u disconnected\n", client->id());
            } else if (type == WS_EVT_DATA) {
                handleMessage((char*)data, len);
            }
        });
        server.addHandler(&ws);

        // Serve web dashboard
        server.serveStatic("/", SPIFFS, "/");
        
        // Start server
        server.begin();
        Serial.printf("[Comms] WebSocket server on http://%s\n", WifiApIp);
    }

    /**
     * @brief Broadcast telemetry as JSON to all connected clients
     */
    void sendTelemetry() {
        if (millis() - lastTelemetryTime < 50) {
            return;  // 20 Hz max
        }

        StaticJsonDocument<512> doc;
        
        // Pose
        Pose p = poseEstimator->getPose();
        doc["pose"]["x"] = (int)p.x;
        doc["pose"]["y"] = (int)p.y;
        doc["pose"]["theta"] = p.theta;

        // Battery
        doc["battery"]["voltage"] = batteryMonitor->getVoltage();
        doc["battery"]["percentage"] = (int)batteryMonitor->getPercentage();

        // Motion state
        doc["moving"] = motionController->isMoving();
        doc["time"] = millis();

        // Serialize and send to all clients
        String json;
        serializeJson(doc, json);
        ws.textAll(json);

        lastTelemetryTime = millis();
    }

    /**
     * @brief Handle incoming WebSocket message with command processing
     */
    void handleMessage(const char* data, size_t len) {
        StaticJsonDocument<256> doc;
        if (deserializeJson(doc, data)) {
            Serial.println("[Comms] JSON parse error");
            return;
        }

        const char* cmd = doc["cmd"];
        if (!cmd) {
            return;
        }

        // ===== MOTION COMMANDS =====
        if (strcmp(cmd, "motion") == 0) {
            float linear = doc["linear"] | 0.0f;    // mm/s
            float angular = doc["angular"] | 0.0f;  // rad/s
            motionController->directMotion(linear, angular);
            Serial.printf("[Comms] Motion: linear=%.1f angular=%.3f\n", linear, angular);
        }

        // ===== STOP COMMAND =====
        else if (strcmp(cmd, "stop") == 0) {
            motionController->stop();
            Serial.println("[Comms] STOP command received");
        }
        
        // ===== SEQUENCE 1: STAIRS =====
        else if (strcmp(cmd, "stairs") == 0) {
            motionController->clearQueue();
            pathPlanner->planStairs();
            motionController->startPathFollow();
        }

        // ===== FA1: SQUARES =====
        else if (strcmp(cmd, "squares") == 0) {
            float startSize = doc["size"] | 50.0f;
            int count = doc["count"] | 3;
            motionController->clearQueue();
            pathPlanner->planSquares(startSize, count);
            motionController->startPathFollow();
        }

        // ===== SEQUENCE 3: COMPASS =====
        else if (strcmp(cmd, "compass") == 0) {
            pathPlanner->planCompassRose(doc["full"] | false);
        }

        // ===== DRAWING COMMANDS =====
        else if (strcmp(cmd, "draw") == 0) {
            const char* shape = doc["shape"];
            processDrawCommand(shape, doc);
        }

        // ===== CONFIG COMMANDS =====
        else if (strcmp(cmd, "config") == 0) {
            processConfigCommand(doc);
        }
    }

private:
    /**
     * @brief Process drawing shape commands
     */
    void processDrawCommand(const char* shape, JsonDocument& doc) {
        if (!pathPlanner || !motionController) {
            Serial.println("[Comms] Drawing system not ready");
            return;
        }

        if (strcmp(shape, "circle") == 0) {
            float radius = doc["radius"] | 50.0f;
            float speed = doc["speed"] | 100.0f;
            motionController->clearQueue();
            pathPlanner->planCircle(0, 0, radius, speed);
            motionController->startPathFollow();
            Serial.printf("[Comms] Drawing circle: radius=%.0f\n", radius);
        }
        else if (strcmp(shape, "triangle") == 0) {
            float size = doc["size"] | 100.0f;
            float speed = doc["speed"] | 100.0f;
            motionController->clearQueue();
            pathPlanner->planTriangle(0, 0, size, size, size/2, -size, speed);
            motionController->startPathFollow();
            Serial.printf("[Comms] Drawing triangle: size=%.0f\n", size);
        }
        else if (strcmp(shape, "rectangle") == 0) {
            float width = doc["width"] | 100.0f;
            float height = doc["height"] | 50.0f;
            float speed = doc["speed"] | 100.0f;
            motionController->clearQueue();
            pathPlanner->planRectangle(0, 0, width, height, speed);
            motionController->startPathFollow();
            Serial.printf("[Comms] Drawing rectangle: %.0f x %.0f\n", width, height);
        }
        else if (strcmp(shape, "spiral") == 0) {
            float innerR = doc["inner"] | 10.0f;
            float outerR = doc["outer"] | 100.0f;
            float speed = doc["speed"] | 100.0f;
            motionController->clearQueue();
            pathPlanner->planSpiral(0, 0, innerR, outerR, speed);
            motionController->startPathFollow();
            Serial.printf("[Comms] Drawing spiral: %.0f to %.0f\n", innerR, outerR);
        }
    }

    /**
     * @brief Process configuration commands
     */
    void processConfigCommand(JsonDocument& doc) {
        const char* param = doc["param"];
        float value = doc["value"];
        // TODO: Update ConfigManager with new parameters
        Serial.printf("[Comms] Config: %s = %.3f\n", param, value);
    }
};

#endif // COMMS_MANAGER_H
```

---

## ✅ Verification

- [ ] WiFi AP broadcasts "RobotWifi"
- [ ] Can connect to 192.168.4.1
- [ ] Telemetry streams as JSON (20 Hz)
- [ ] Motion commands received and executed immediately
- [ ] Drawing commands queue waypoints and execute
- [ ] Stop command halts all motion

---

## 📋 Command Protocol

### Motion Command
```json
{
  "cmd": "motion",
  "linear": 50.0,    // mm/s (positive = forward)
  "angular": 0.5     // rad/s (positive = counter-clockwise)
}
```

### Stop Command
```json
{"cmd": "stop"}
```

### Drawing Commands
```json
{
  "cmd": "draw",
  "shape": "circle",
  "radius": 50.0,
  "speed": 100.0
}
```

Shapes supported: `circle`, `triangle`, `rectangle`, `spiral`

### Config Command
```json
{
  "cmd": "config",
  "param": "Kp_linear",
  "value": 0.5
}
```

---

## 🔄 Next Steps

1. **Step 13**: Web dashboard (HTML/CSS/JS) - COMPLETE
2. **Step 14**: Integration testing - Connect all components
