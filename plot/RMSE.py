import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

# ====== Datei-Namen ======
files = {
    "KF": "kf_prediction.csv",
    "EKF": "ekf.csv",
    "PF": "prediction.csv",
    "Odom": "odom.csv"
}

# ====== Dateien laden und prüfen ======
data = {}
for name, path in files.items():
    if not os.path.exists(path):
        raise FileNotFoundError(f"{path} nicht gefunden")
    df = pd.read_csv(path).drop_duplicates(subset='t')
    data[name] = df

# ====== Interpolation auf Odom-Zeitbasis ======
odom = data["Odom"]
rmse_results = {}

for label in ["KF", "EKF", "PF"]:
    df = data[label]
    df_interp = df.set_index('t').reindex(odom['t'], method='nearest').reset_index()

    # RMSE
    rmse_x = np.sqrt(np.mean((df_interp['x'] - odom['x'])**2))
    rmse_y = np.sqrt(np.mean((df_interp['y'] - odom['y'])**2))
    rmse_total = np.sqrt(rmse_x**2 + rmse_y**2)
    rmse_results[label] = (rmse_x, rmse_y, rmse_total)

    # Speichern für Plot
    data[label + "_interp"] = df_interp

# ====== Ausgabe der RMSE-Werte ======
print("📊 RMSE-Vergleich (in Metern):")
for label, (rx, ry, rtotal) in rmse_results.items():
    print(f"{label}: RMSE_x = {rx:.4f}, RMSE_y = {ry:.4f}, Gesamt = {rtotal:.4f}")

# ====== Plot ======
plt.figure(figsize=(10, 6))
plt.plot(odom['x'], odom['y'], label='Ground Truth (odom)', linewidth=2, color='black')

for label in ["KF", "EKF", "PF"]:
    interp = data[label + "_interp"]
    plt.plot(interp['x'], interp['y'], label=f'{label} (RMSE={rmse_results[label][2]:.3f} m)', linestyle='--')

plt.title("Trajektorienvergleich")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
