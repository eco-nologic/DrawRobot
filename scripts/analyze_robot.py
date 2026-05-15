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
    if df.empty:
        print(f"⚠️ Le fichier {latest_file} est vide. Assurez-vous que log_telemetry.py a bien reçu des données du robot.")
        return

    df['time_s'] = (df['timestamp_ms'] - df['timestamp_ms'].iloc[0]) / 1000.0
    
    # MATH: Intégration de la distance parcourue (lmes)
    dist_inc = np.sqrt(np.diff(df['x_mm'])**2 + np.diff(df['y_mm'])**2)
    dist_cum = np.cumsum(np.insert(dist_inc, 0, 0))

    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle(f"Rapport de Validation (travail.pdf) - {latest_file}", fontsize=16)

    # 1. Courbe lmes vs lth (p.5 du PDF)
    # DEFENSE: "Comment validez-vous la précision de 1cm ?"
    # ANSWER: En traçant la distance mesurée (lmes) par rapport à la commande envoyée (lth).
    # La proximité avec la droite 1:1 confirme le respect de l'exigence ET1.2.
    ax1.scatter(df['target_l'], dist_cum, color='blue', alpha=0.5, label='Mesures')
    ax1.plot([0, df['target_l'].max()], [0, df['target_l'].max()], 'r--', label='Idéal (1:1)')
    ax1.set_title("Séquence 1 : lmes [cm] vs lth [cm]")
    ax1.set_xlabel("Consigne lth (mm)")
    ax1.set_ylabel("Mesuré lmes (mm)")
    ax1.grid(True)

    # 2. Courbe thetames vs thetath (p.5 du PDF)
    # DEFENSE: "Comment validez-vous les angles de 90° ?"
    # ANSWER: Cette courbe compare l'angle réel (fusion IMU) à l'angle cible. 
    # Les paliers à 90° et 180° valident la précision angulaire (ET1.3).
    ax2.scatter(df['target_theta'], df['heading_deg'], color='red', alpha=0.5)
    ax2.plot([-180, 180], [-180, 180], 'k--')
    ax2.set_title("Séquence 1 : θmes [°] vs θth [°]")
    ax2.set_xlabel("Consigne θth (°)")
    ax2.set_ylabel("Mesuré θmes (°)")
    ax2.grid(True)

    # 3. Cap [°] vs t [s] (p.5 du PDF)
    ax3.plot(df['time_s'], df['heading_deg'], color='green', label='Cap fusionné')
    ax3.set_title("Acquisition temporelle du Cap")
    ax3.set_xlabel("Temps (s)")
    ax3.set_ylabel("Degrés (°)")
    ax3.grid(True)

    # 4. Erreur de fermeture dmes vs r (p.7 du PDF)
    # MATH: Distance entre le point final et initial.
    error_closure = np.sqrt((df['x_mm'].iloc[-1] - df['x_mm'].iloc[0])**2 + 
                            (df['y_mm'].iloc[-1] - df['y_mm'].iloc[0])**2)
    ax4.plot(df['target_r'], np.full_like(df['target_r'], error_closure), 'o-', color='orange')
    ax4.set_title("Séquence 2 : dmes [cm] vs r [cm]")
    ax4.set_xlabel("Rayon théorique r (mm)")
    ax4.set_ylabel("Erreur de fermeture (mm)")
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