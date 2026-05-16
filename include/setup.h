#ifndef SETUP_ROBOT_H
#define SETUP_ROBOT_H

/**
 * @brief Setup helper functions to initialize hardware and peripherals.
 * Moved from main.cpp to improve code organization.
 */

void setupWifi();
void setupGpio();
void setupSerial();
void verifyMotorHardware();
void verifyNavigationLogic();

#endif