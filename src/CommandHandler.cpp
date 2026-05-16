#include "CommandHandler.h"
#include <Arduino.h>

CommandHandler::CommandHandler(PathPlanner& planner, DriveTrain& drive)
    : _planner(planner), _drive(drive) {}

void CommandHandler::processJSONCommand(uint8_t* data, size_t len) {
    JsonDocument doc; 
    DeserializationError error = deserializeJson(doc, (char*)data, len);

    if (error) {
        Serial.printf("[Cmd] JSON Parse Error: %s\n", error.c_str());
        return;
    }

    const char* cmd = doc["cmd"] | "";
    if (strlen(cmd) > 0) {
        Serial.printf("[Cmd] Received: %s\n", cmd);

        // Manual D-Pad Controls
        if (strcmp(cmd, "FORWARD") == 0)   { _planner.stop(); _drive.setVelocity(0.6f, 0.0f); }
        else if (strcmp(cmd, "BACKWARD") == 0) { _planner.stop(); _drive.setVelocity(-0.6f, 0.0f); }
        else if (strcmp(cmd, "TURN_LEFT") == 0) { _planner.stop(); _drive.setVelocity(0.0f, 1.2f); }
        else if (strcmp(cmd, "TURN_RIGHT") == 0) { _planner.stop(); _drive.setVelocity(0.0f, -1.2f); }
        else if (strcmp(cmd, "STOP") == 0)      { _planner.stop(); _drive.stop(); }

        // Routage des commandes vers les séquences du PathPlanner
        else if (strcmp(cmd, "stairs") == 0)      { _planner.stop(); _planner.startStairs(); }
        else if (strcmp(cmd, "circle") == 0)      { _planner.startCircle(doc["radius"] | 50.0f); } 
        else if (strcmp(cmd, "squares") == 0)     { _planner.startSquares(doc["size"] | 50.0f, doc["count"] | 3); }
        else if (strcmp(cmd, "north") == 0)       { _planner.stop(); _planner.startNorthArrow(); }
        else if (strcmp(cmd, "calibrateMag") == 0) { _planner.stop(); _planner.startCalibrationSequence(); }
        else if (strcmp(cmd, "balance") == 0)      { _planner.startBalance(); }
        else if (strcmp(cmd, "stop") == 0)         { _planner.stop(); _drive.stop(); }

        // PID Configuration
        else if (strcmp(cmd, "config") == 0) {
            // DEFENSE: "Comment avez-vous réglé vos PID ?"
            // ANSWER: En envoyant des valeurs via WebSocket et en observant 
            // la réponse du robot en temps réel sur le dashboard.
            _planner.getMotionController().setPidParams(
                doc["lkp"] | 0.5f, doc["lki"] | 0.1f, doc["lkd"] | 0.01f,
                doc["akp"] | 0.8f, doc["aki"] | 0.2f, doc["akd"] | 0.1f
            );
        }
    }
}