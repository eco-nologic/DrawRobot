#include <Arduino.h>
#include <Wire.h>
#include "Config.h"

// UNCOMMENT the line below only if you want to run the Scanner instead of the Robot code.
// #define RUN_I2C_SCANNER

#ifdef RUN_I2C_SCANNER
// Temporary I2C scanner - upload this to diagnose the problem
void setup() {
    Serial.begin(115200);
    delay(500);
    
    Serial.println("\n[I2C Scanner] Starting bus recovery...");
    
    // Bus recovery
    pinMode(Config::SDA, OUTPUT);
    pinMode(Config::SCL, OUTPUT);
    digitalWrite(Config::SDA, HIGH);
    
    for (int i = 0; i < 9; i++) {
        digitalWrite(Config::SCL, LOW);
        delayMicroseconds(5);
        digitalWrite(Config::SCL, HIGH);
        delayMicroseconds(5);
    }
    
    pinMode(Config::SDA, INPUT_PULLUP);
    pinMode(Config::SCL, INPUT_PULLUP);
    delay(20);
    
    Serial.println("[I2C Scanner] Initializing Wire...");
    if (!Wire.begin(Config::SDA, Config::SCL)) {
        Serial.println("[I2C Scanner] ERROR: Wire.begin failed!");
        return;
    }
    
    Wire.setClock(Config::I2C_FREQ);
    Serial.printf("[I2C Scanner] Clock set to %d Hz\n", Config::I2C_FREQ);
    
    Serial.println("\n[I2C Scanner] Scanning addresses 0x00-0x7F...");
    int found = 0;
    
    for (byte addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        byte error = Wire.endTransmission();
        
        if (error == 0) {
            Serial.printf("[I2C] Device found at 0x%02X\n", addr);
            found++;
        }
    }
    
    Serial.printf("\n[I2C Scanner] Scan complete. Found %d device(s).\n", found);
    Serial.println("[I2C Scanner] Expected addresses:");
    Serial.println("  LSM6DS3: 0x6A");
    Serial.println("  LIS3MDL: 0x1E (or 0x1C with ALT address pin)");
}

void loop() {
    delay(1000);
    Serial.println("[I2C Scanner] Running... (waiting for input to restart)");
}
#endif
