import asyncio
import platform
import sys
import subprocess
import csv
import time
import os
from bleak import BleakClient, BleakScanner

# --- Détection et installation automatique des dépendances ---
try:
    import bleak
    import serial.tools.list_ports # Pour la liste des ports en cas d'erreur
except ImportError:
    print("[Python] Modules 'bleak' ou 'pyserial' manquants. Installation en cours...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "bleak", "pyserial"])
    import bleak
    import serial.tools.list_ports

# --- Configuration BLE ---
# Ces UUIDs doivent correspondre à ceux définis dans BluetoothManager.cpp de votre firmware
DEVICE_NAME = "GiRobot_BLE"
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

# --- Configuration CSV ---
CSV_HEADERS = [
    "timestamp_ms", "x_mm", "y_mm", "heading_deg", "battery_v",
    "accel_x", "accel_y", "accel_z",
    "gyro_x", "gyro_y", "gyro_z",
    "mag_x", "mag_y", "mag_z",
    "target_l", "target_theta", "target_r"
]

# Ensure the Output directory exists for data storage
if not os.path.exists("Output"):
    os.makedirs("Output")

# MATH: int(time.time()) fournit un timestamp Unix unique.
FILENAME = os.path.join("Output", f"robot_data_{int(time.time())}.csv")

async def run_ble_client():
    print(f"Recherche du robot BLE : {DEVICE_NAME}...")
    # MATH/LOGIC: On augmente le timeout à 10s pour laisser le temps à la pile Bluetooth 
    # Windows de découvrir l'appareil, surtout en environnement bruité.
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)

    if not device:
        print(f"❌ Appareil '{DEVICE_NAME}' non trouvé. Assurez-vous que l'ESP32 est allumé et diffuse.")
        # Tente de lister les ports COM pour aider au diagnostic si BLE ne fonctionne pas
        try:
            ports = list(serial.tools.list_ports.comports())
            if ports:
                print("\nPorts COM détectés (pour diagnostic si BLE échoue) :")
                for p in ports:
                    print(f"  - {p.device} ({p.description})")
            else:
                print("\nAucun port COM détecté. Vérifiez vos connexions.")
        except Exception:
            pass # pyserial peut ne pas être installé
        return

    print(f"Appareil trouvé à l'adresse : {device.address}. Connexion...")

    async with BleakClient(device) as client:
        if not client.is_connected:
            print(f"❌ Échec de la connexion à {DEVICE_NAME}.")
            return

        print(f"✅ Connecté à {DEVICE_NAME}!")

        points_count = 0
        with open(FILENAME, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(CSV_HEADERS)
            print(f"Enregistrement des données dans {FILENAME}")

            def notification_handler(sender, data):
                nonlocal points_count
                try:
                    decoded_data = data.decode('utf-8')
                    # DEFENSE: "Quel est le format de la trame Bluetooth ?"
                    # ANSWER: Une chaîne formatée (A:...|G:...|M:...|H:...|B:...|T:...|Lth:...|Ath:...|Rth:...) 
                    # pour minimiser la bande passante par rapport au JSON tout en restant lisible.
                    parts = decoded_data.split('|')
                    
                    # Parsing des données
                    accel_data = [float(x) for x in parts[0].split(':')[1].split(',')]
                    gyro_data = [float(x) for x in parts[1].split(':')[1].split(',')]
                    mag_data = [float(x) for x in parts[2].split(':')[1].split(',')]
                    heading = float(parts[3].split(':')[1])
                    battery_v = float(parts[4].split(':')[1])
                    timestamp_ms = int(parts[5].split(':')[1])
                    # Nouveaux champs pour la validation travail.pdf
                    target_l = float(parts[6].split(':')[1])
                    target_theta = float(parts[7].split(':')[1])
                    target_r = float(parts[8].split(':')[1])

                    row = [timestamp_ms, 0.0, 0.0, heading, battery_v] + accel_data + gyro_data + mag_data + [target_l, target_theta, target_r]
                    csv_writer.writerow(row)
                    points_count += 1
                    
                    # Feedback console et sauvegarde physique immédiate
                    if points_count % 10 == 0:
                        print(f"📡 Recu: {points_count} points | T={timestamp_ms}ms | H={heading:.2f}° | Bat={battery_v:.1f}V")
                        csvfile.flush()

                except Exception as e:
                    print(f"⚠️ Erreur de traitement de la notification: {e} - Données brutes: {data.hex()}")

            print(f"Abonnement à la caractéristique: {CHARACTERISTIC_UUID}")
            await client.start_notify(CHARACTERISTIC_UUID, notification_handler)

            print("Capture des données démarrée. Appuyez sur Ctrl+C pour arrêter.")
            while client.is_connected:
                await asyncio.sleep(1) # Maintient la connexion active

        print("Déconnecté de l'appareil BLE.")

async def main():
    try:
        await run_ble_client()
    except asyncio.CancelledError:
        print("\nClient BLE arrêté par l'utilisateur.")
    except Exception as e:
        print(f"Une erreur inattendue est survenue: {e}")

if __name__ == "__main__":
    # S'assurer que le script est exécuté avec Python 3.7+
    if sys.version_info < (3, 7):
        print("Ce script nécessite Python 3.7 ou plus récent.")
        sys.exit(1)

    # DEFENSE: "Pourquoi avoir supprimé WindowsSelectorEventLoopPolicy ?"
    # ANSWER: Cette politique est obsolète depuis Python 3.12. Bleak sur Windows 10/11 
    # utilise désormais nativement la boucle Proactor pour une meilleure gestion du Bluetooth.

    asyncio.run(main())