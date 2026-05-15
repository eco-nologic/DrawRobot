#ifndef TELEMETRY_PACKET_H
#define TELEMETRY_PACKET_H

// DEFENSE: "Pourquoi regrouper toutes les données dans une seule structure ?"
// ANSWER: Pour simplifier le passage des données entre les modules (ex: PoseEstimator vers BluetoothManager)
// et pour faciliter le formatage de la trame de télémétrie (JSON ou chaîne compacte).
struct TelemetryPacket {
    // Pose du robot
    float robotX;
    float robotY;
    float robotHeading;
    // Pose "fantôme" (pure odométrie)
    float ghostX;
    float ghostY;
    float ghostHeading;
    // Moteurs
    float leftWheelSpeed;
    float rightWheelSpeed;
    long leftWheelSteps;
    long rightWheelSteps;
    // IMU (données brutes)
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ;
    // Autres
    float batteryVoltage;
    bool isMoving;
    bool isCalibrated; // État de la calibration IMU
    int waypointIndex;
    float targetX, targetY;
    float bearingToTarget; // Cap vers la cible
};

#endif // TELEMETRY_PACKET_H