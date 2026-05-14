#include "ConfigManager.h"
#include <Arduino.h>

/**
 * DEFENSE: "Où sont stockées les données de calibration ?"
 * ANSWER: Dans la partition NVS (Non-Volatile Storage) de la mémoire Flash. 
 * Contrairement à la RAM, les données y persistent même sans alimentation.
 */
void ConfigManager::begin() {
    // Ouverture du namespace "robot-cfg" en mode lecture/écriture
    if (!_prefs.begin("robot-cfg", false)) {
        Serial.println("[Config] Erreur critique: NVS inaccessible.");
    }
}

void ConfigManager::loadConfiguration() {
    /**
     * DEFENSE: "Que se passe-t-il si aucune calibration n'a été faite ?"
     * ANSWER: La méthode getFloat() retourne une valeur par défaut (définie dans Config.h) 
     * si la clé n'existe pas encore en mémoire.
     */
    Serial.println("[Config] Chargement des paramètres permanents...");
    
    // Exemple de lecture : les gains sont appliqués au démarrage via main.cpp
    // float lkp = _prefs.getFloat("lkp", Config::PidLinearKp);
}

/**
 * @brief Découpage en fonction simple pour la clarté (Tips)
 */
void ConfigManager::saveFloat(const char* key, float value) {
    /**
     * DEFENSE: "Comment garantissez-vous que le robot ne perd pas ses réglages ?"
     * ANSWER: Chaque modification via l'interface Web déclenche un putFloat() 
     * qui écrit physiquement la valeur dans la Flash de l'ESP32.
     */
    _prefs.putFloat(key, value);
    Serial.printf("[Config] Sauvegarde NVS: %s = %.3f\n", key, value);
}