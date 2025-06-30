import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt

# CSV-Dateien einlesen
odom_df = pd.read_csv('../odom.csv', delimiter=':', header=None, on_bad_lines='skip')
pf_df = pd.read_csv('../prediction.csv', delimiter=':', header=None, on_bad_lines='skip')
kf_df = pd.read_csv('../kf_prediction.csv', delimiter=':', header=None, on_bad_lines='skip')

# Hilfsfunktion zur Extraktion der Positionen aus den CSV-Daten
def extract_positions(df):
    xs, ys = [], []
    for i, row in df.iterrows():
        if 'position' in str(row[0]):
            x = float(str(row[1]).split(':')[-1])
            y = float(str(df.iloc[i+1][1]).split(':')[-1])
            xs.append(x)
            ys.append(y)
    return np.array(xs), np.array(ys)

# Positionen extrahieren
odom_x, odom_y = extract_positions(odom_df)
pf_x, pf_y = extract_positions(pf_df)
kf_x, kf_y = extract_positions(kf_df)

# Kürzen auf gleiche Länge
min_len = min(len(odom_x), len(pf_x), len(kf_x))
odom_x, odom_y = odom_x[:min_len], odom_y[:min_len]
pf_x, pf_y = pf_x[:min_len], pf_y[:min_len]
kf_x, kf_y = kf_x[:min_len], kf_y[:min_len]

# RMSE berechnen
def rmse(x1, y1, x2, y2):
    return sqrt(np.mean((x1 - x2)**2 + (y1 - y2)**2))

rmse_pf = rmse(odom_x, odom_y, pf_x, pf_y)
rmse_kf = rmse(odom_x, odom_y, kf_x, kf_y)

print(f"RMSE Particle Filter: {rmse_pf:.4f} m")
print(f"RMSE Kalman Filter:   {rmse_kf:.4f} m")

# Plot
plt.figure(figsize=(8, 6))
plt.plot(odom_x, odom_y, label='Ground Truth (odom)', linewidth=2)
plt.plot(pf_x, pf_y, label='Particle Filter', linestyle='--')
plt.plot(kf_x, kf_y, label='Kalman Filter', linestyle=':')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('Trajectory Comparison')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('trajectory_comparison.png')
plt.show()
