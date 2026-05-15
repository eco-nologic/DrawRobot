#ifndef COMMS_MANAGER_H
#define COMMS_MANAGER_H

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "PathPlanner.h"
#include "TelemetryPacket.h"

/**
 * DEFENSE: "Pourquoi utiliser des WebSockets et du JSON ?"
 * ANSWER: Les WebSockets permettent une communication bi-directionnelle en temps réel 
 * sans le surpoids du HTTP. Le JSON est standard, léger et facile à traiter en JavaScript.
 */
class CommsManager {
public:
    CommsManager(PathPlanner& planner);

    void begin();
    
    // Envoie l'état du robot (X, Y, Cap, Batterie) au tableau de bord
    void broadcastTelemetry(const TelemetryPacket& packet);

private:
    AsyncWebServer _server;
    AsyncWebSocket _ws;
    PathPlanner& _planner;

    void onEvent(AsyncWebSocket* s, AsyncWebSocketClient* c, AwsEventType t, void* arg, uint8_t* data, size_t len);
};
#endif