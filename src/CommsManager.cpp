#include "CommsManager.h"
#include "Config.h"
#include <LittleFS.h>

// Constructeur: Initialise le serveur sur le port 80 et définit le point d'accès WebSocket
CommsManager::CommsManager(PathPlanner& planner) 
    : _server(80), _ws("/ws"), _planner(planner) {}

void CommsManager::begin() {
    // Montage du système de fichiers pour servir le Dashboard Web
    // formatOnFail = true permet d'initialiser la partition si elle est corrompue
    if(!LittleFS.begin(true)){
        Serial.println("[Comms] Failed to mount LittleFS");
    } else {
        Serial.println("[Comms] LittleFS mounted successfully.");
        
        // Debug: Liste les fichiers présents pour valider l'upload
        File root = LittleFS.open("/");
        File file = root.openNextFile();
        while(file){
            Serial.printf("  File: %s, Size: %d bytes\n", file.name(), file.size());
            file = root.openNextFile();
        }
    }

    // Liaison de l'événement de réception au gestionnaire de classe via une lambda
    _ws.onEvent([this](AsyncWebSocket* s, AsyncWebSocketClient* c, AwsEventType t, void* arg, uint8_t* data, size_t len) {
        this->onEvent(s, c, t, arg, data, len);
    });

    // IMPORTANT: Toujours ajouter les handlers spécifiques (WebSocket, API) AVANT le serveur de fichiers statiques.
    // Sinon, le serveur statique risque d'intercepter la requête s'il est mappé sur la racine.
    _server.addHandler(&_ws);

    // Route explicite pour la racine - garantit que l'index est toujours servi correctement
    // sans dépendre du comportement de "setDefaultFile" qui varie selon les navigateurs.
    _server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(LittleFS, "/index.html", "text/html");
    });

    // Sert les autres fichiers statiques (style.css, script.js)
    // On mappe sans le "setDefaultFile" pour éviter les ambiguïtés de chemin sur LittleFS.
    _server.serveStatic("/", LittleFS, "/");

    _server.begin();
}

void CommsManager::onEvent(AsyncWebSocket* s, AsyncWebSocketClient* c, AwsEventType t, void* arg, uint8_t* data, size_t len) {
    // DEFENSE: "Comment gérez-vous les commandes asynchrones ?"
    // ANSWER: On intercepte l'événement WS_EVT_DATA. Le format JSON permet d'extraire les paramètres facilement.
    if (t == WS_EVT_DATA) {
        JsonDocument doc; 
        DeserializationError error = deserializeJson(doc, (char*)data, len);

        if (!error) {
            const char* cmd = doc["cmd"];
            // Routage des commandes vers les séquences du PathPlanner
            if (strcmp(cmd, "stairs") == 0) _planner.startStairs();
            if (strcmp(cmd, "circle") == 0) _planner.startCircle(doc["radius"] | 50.0f);
            if (strcmp(cmd, "squares") == 0) _planner.startSquares(doc["size"] | 50.0f, doc["count"] | 3);
            if (strcmp(cmd, "north") == 0) _planner.startNorthArrow();
            if (strcmp(cmd, "calibrateMag") == 0) _planner.startCalibrationSequence();
            if (strcmp(cmd, "stop") == 0) _planner.stop(); 

            // --- Nouvelle commande de configuration ---
            if (strcmp(cmd, "config") == 0) {
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
}

void CommsManager::broadcastTelemetry(float x, float y, float heading, float battery) {
    // DEFENSE: "Pourquoi utiliser du JSON pour la télémétrie ?"
    // ANSWER: C'est un format texte léger que le JavaScript du dashboard peut parser nativement en un seul appel.
    if (_ws.count() > 0) { // On n'envoie les données que si un client est connecté (économie CPU)
        JsonDocument doc;
        doc["x"] = x; doc["y"] = y; doc["h"] = heading; doc["b"] = battery;
        
        doc["calib"] = false; // Note: Update signature to pass actual status if available
        String output;
        serializeJson(doc, output);
        _ws.textAll(output); // Diffusion temps-réel à tous les clients connectés
    }
}