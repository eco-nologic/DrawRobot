import serial
import json
import csv
import time
import os

# CONFIGURATION
# DEFENSE: "Comment communiquez-vous avec le robot depuis le PC ?"
# ANSWER: Le PC se connecte au port série virtuel créé par le Bluetooth (COM port).
# Le script Python parse les lignes JSON et les enregistre pour analyse.
PORT = 'COM10'  # Modifier selon votre port Bluetooth (ex: /dev/rfcomm0 sur Linux)
BAUD = 115200

# Ensure the Output directory exists for data storage
if not os.path.exists("Output"):
    os.makedirs("Output")

# Math: int(time.time()) provides a unique Unix Epoch timestamp (integer).
# This ensures that every run creates a unique file in the Output folder without overwriting data.
FILENAME = os.path.join("Output", f"robot_data_{int(time.time())}.csv")

print(f"Connexion au robot sur {PORT}...")
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Enregistrement dans {FILENAME}. Appuyez sur Ctrl+C pour arreter.")
    
    with open(FILENAME, mode='w', newline='') as file:
        writer = csv.writer(file)
        # En-tetes correspondant aux exigences du rapport (lmes, theta, t)
        writer.writerow(['timestamp_ms', 'x_mm', 'y_mm', 'heading_deg', 'battery_v'])
        
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                try:
                    # Parsing des donnees envoyees par BluetoothManager.cpp
                    data = json.loads(line)
                    writer.writerow([
                        data['t'], 
                        data['x'], 
                        data['y'], 
                        data['h'], 
                        data['b']
                    ])
                    # Feedback console pour surveiller le robot pendant le tracé
                    print(f"T={data['t']}ms | X={data['x']:.1f} Y={data['y']:.1f} | Bat={data['b']:.1f}V")
                except json.JSONDecodeError:
                    pass # Ignorer les lignes de debug non-JSON
                    
except serial.SerialException as e:
    print(f"Erreur de connexion : {e}")
    print("Assurez-vous que le robot est apparie et que le port COM est correct.")
except KeyboardInterrupt:
    print("\nCapture terminee. Fichier enregistre.")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()