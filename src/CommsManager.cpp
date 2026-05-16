#include "CommsManager.h"
#include "Config.h"
#include <LittleFS.h>

// Constructeur: Initialise le serveur sur le port 80 et définit le point d'accès WebSocket
CommsManager::CommsManager(CommandHandler& handler) 
    : _server(80), _ws("/ws"), _commandHandler(handler) {}

void CommsManager::begin() {
    // Liaison de l'événement de réception au gestionnaire de classe via une lambda
    _ws.onEvent([this](AsyncWebSocket* s, AsyncWebSocketClient* c, AwsEventType t, void* arg, uint8_t* data, size_t len) {
        if (t == WS_EVT_CONNECT) Serial.printf("[WS] Client %u connected. Total: %u\n", c->id(), s->count());
        this->onEvent(s, c, t, arg, data, len);
    });

    // IMPORTANT: Toujours ajouter les handlers spécifiques (WebSocket, API) AVANT le serveur de fichiers statiques.
    // Sinon, le serveur statique risque d'intercepter la requête s'il est mappé sur la racine.
    _server.addHandler(&_ws);

    // Handle favicon gracefully to avoid VFS errors
    _server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(204);
    });

    // Explicitly handle index to prevent 404s on root
    auto handleIndex = [](AsyncWebServerRequest *request) {
        request->send(LittleFS, "/index.html", "text/html");
    };
    _server.on("/", HTTP_GET, handleIndex);
    _server.on("/index.html", HTTP_GET, handleIndex);

    // Serve static assets
    _server.serveStatic("/", LittleFS, "/");

    _server.begin();
}

void CommsManager::onEvent(AsyncWebSocket* s, AsyncWebSocketClient* c, AwsEventType t, void* arg, uint8_t* data, size_t len) {
    // DEFENSE: "Comment gérez-vous les commandes asynchrones ?"
    // ANSWER: On intercepte l'événement WS_EVT_DATA. Le format JSON permet d'extraire les paramètres facilement.
    if (t == WS_EVT_DATA) {
        // Forward the data to the central CommandHandler
        _commandHandler.processJSONCommand(data, len);
    }
}

void CommsManager::broadcastTelemetry(const TelemetryPacket& packet) {
    // DEFENSE: "Pourquoi utiliser du JSON pour la télémétrie ?"
    // ANSWER: C'est un format texte léger que le JavaScript du dashboard peut parser nativement en un seul appel.
    if (_ws.count() > 0) { // On n'envoie les données que si un client est connecté (économie CPU)
        _ws.cleanupClients();

        JsonDocument doc;
        doc["x"] = packet.robotX;
        doc["y"] = packet.robotY;
        doc["h"] = packet.robotHeading;
        doc["b"] = packet.batteryVoltage;
        doc["t"] = millis(); // Temps pour l'acquisition temporelle (Requirement p.17)

        // Ajout des données IMU brutes pour le debug et l'analyse
        doc["ax"] = packet.accelX;
        doc["ay"] = packet.accelY;
        doc["az"] = packet.accelZ;
        doc["gx"] = packet.gyroX;
        doc["gy"] = packet.gyroY;
        doc["gz"] = packet.gyroZ;
        doc["mx"] = packet.magX;
        doc["my"] = packet.magY;
        doc["mz"] = packet.magZ;

        // Données d'odométrie (ghost)
        doc["gx_o"] = packet.ghostX;
        doc["gy_o"] = packet.ghostY;
        doc["gh_o"] = packet.ghostHeading;

        // Données moteurs
        doc["ls"] = packet.leftWheelSpeed;
        doc["rs"] = packet.rightWheelSpeed;
        doc["lc"] = packet.leftWheelSteps;
        doc["rc"] = packet.rightWheelSteps;
        
        doc["calib"] = packet.isCalibrated; 
        String output;
        serializeJson(doc, output);
        _ws.textAll(output); // Diffusion temps-réel à tous les clients connectés
    }
}