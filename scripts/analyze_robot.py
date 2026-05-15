import sys
import subprocess

# Détection et installation automatique des dépendances
required_packages = {
    "pandas": "pandas",
    "matplotlib": "matplotlib",
    "numpy": "numpy"
}

for module, package in required_packages.items():
    try:
        __import__(module)
    except ImportError:
        print(f"[Python] Module '{module}' manquant. Installation de '{package}'...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", package])

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import glob
import os
import time

def generate_validation_report():
    """
    @brief Analyse les logs CSV et génère les graphiques de validation.
    """
    # DEFENSE: "Où sont stockées les données brutes ?"
    # ANSWER: Dans le dossier 'Output/', ce qui permet de séparer le code source des mesures expérimentales.
    list_of_files = glob.glob(os.path.join('Output', 'robot_data_*.csv'))
    if not list_of_files:
        print("No log files found.")
        return
    
    latest_file = max(list_of_files, key=os.path.getctime)
    print(f"Analyse du fichier : {latest_file}")
    
    df = pd.read_csv(latest_file)

    # DEFENSE: "Comment gérez-vous les fichiers de logs vides ?"
    # ANSWER: On vérifie si le DataFrame est vide après lecture. Si le fichier ne contient que des en-têtes, 
    # on arrête l'analyse proprement pour éviter une erreur d'indexation (IndexError) sur le calcul du temps.
    if df.empty:
        print(f"⚠️ Le fichier {latest_file} est vide. Assurez-vous que log_telemetry.py a bien reçu des données du robot.")
        return

    df['time_s'] = (df['timestamp_ms'] - df['timestamp_ms'].iloc[0]) / 1000.0

    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle(f"DrawRobot Validation Report - {latest_file}", fontsize=16)

    # 1. XY Trajectory (ET1.2 & ET2.3)
    ax1.plot(df['x_mm'], df['y_mm'], label='Measured Path', color='blue')
    ax1.set_title("Spatial Acquisition (XY)")
    ax1.set_xlabel("X (mm)")
    ax1.set_ylabel("Y (mm)")
    ax1.axis('equal')
    ax1.grid(True)

    # 2. Heading Stability (ET3.1)
    ax2.plot(df['time_s'], df['heading_deg'], label='Measured Heading', color='red')
    ax2.set_title("Orientation (Heading) over Time")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Degrees")
    ax2.grid(True)

    # 3. Distance Accumulation (Validation lmes vs lth)
    # DEFENSE: "Comment calculez-vous la distance totale parcourue (lmes) ?"
    # ANSWER: On applique le théorème de Pythagore (racine de la somme des carrés des deltas X et Y) 
    # entre chaque échantillon de position, puis on utilise np.cumsum() pour l'intégration discrète.
    dist_inc = np.sqrt(np.diff(df['x_mm'])**2 + np.diff(df['y_mm'])**2)
    dist_cum = np.cumsum(np.insert(dist_inc, 0, 0))
    ax3.plot(df['time_s'], dist_cum, label='Measured Distance', color='green')
    ax3.set_title("Cumulative Distance")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("mm")
    ax3.grid(True)

    # 4. Battery Performance
    ax4.plot(df['time_s'], df['battery_v'], label='Voltage', color='orange')
    ax4.set_title("Energy Monitoring")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Volts")
    ax4.grid(True)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    # Final validation calculation
    # DEFENSE: "Comment calculez-vous l'erreur de fermeture ?"
    # ANSWER: C'est la distance euclidienne entre le premier point (0,0) et le dernier point capturé.
    # Elle permet de quantifier la dérive cumulée (drift) sur l'ensemble du trajet.
    error_closure = np.sqrt((df['x_mm'].iloc[-1] - df['x_mm'].iloc[0])**2 + 
                            (df['y_mm'].iloc[-1] - df['y_mm'].iloc[0])**2)
    
    print(f"--- VALIDATION RESULTS ---")
    print(f"Total distance: {dist_cum[-1]:.2f} mm")
    print(f"Closure error: {error_closure:.2f} mm")
    
    # Sauvegarde automatique du rapport dans Output
    if not os.path.exists("Output"):
        os.makedirs("Output")
    report_file = os.path.join("Output", f"validation_report_{int(time.time())}.png")
    plt.savefig(report_file)
    print(f"📊 Rapport sauvegardé dans: {report_file}")
    
    plt.show()

if __name__ == "__main__":
    generate_validation_report()