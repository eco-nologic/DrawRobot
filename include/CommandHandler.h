#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <ArduinoJson.h>
#include "PathPlanner.h"
#include "DriveTrain.h"

/**
 * @class CommandHandler
 * @brief Centralizes command processing logic for all transports (WiFi, BLE).
 * 
 * This class follows the Principle of Single Responsibility by separating 
 * the "What to do" (Command Logic) from the "How it arrived" (Transport).
 */
class CommandHandler {
public:
    CommandHandler(PathPlanner& planner, DriveTrain& drive);
    void processJSONCommand(uint8_t* data, size_t len);

private:
    PathPlanner& _planner;
    DriveTrain& _drive;
};

#endif