import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt

# Daten einlesen mit Zeitstempel
odom = pd.read_csv('odom.csv')
pf = pd.read_csv('prediction.csv')
kf = pd.read_csv('kf_prediction.csv')
ekf = pd.read_csv('ekf.csv')

# Interpolation auf Zeitbasis (Odom als Referenz)
pf_interp = pf.set_index('t').reindex(odom['t'], method='nearest').reset_index()
kf_interp = kf.set_index('t').reindex(odom['t'], method='nearest').reset_index()
ekf_interp = ekf.set_index('t').reindex(odom['t'], method='nearest').reset_index()

# RMSE-Funktion
def rmse(gt, est):
    return sqrt(np.mean((gt['x'] - est['x'])**2 + (gt['y'] - est['y'])**2))

# RMSE berechnen
rmse_pf = rmse(odom, pf_interp)
rmse_kf = rmse(odom, kf_interp)
rmse_ekf = rmse(odom, ekf_interp)


print(f"📊 RMSE Particle Filter: {rmse_pf:.4f} m")
print(f"📊 RMSE Kalman Filter:   {rmse_kf:.4f} m")
print(f"📊 RMSE Extended Kalman Filter: {rmse_ekf:.4f} m")

# Plot
plt.figure(figsize=(8, 6))
plt.plot(odom['x'], odom['y'], label='Ground Truth (odom)', linewidth=2)
plt.plot(pf_interp['x'], pf_interp['y'], label='Particle Filter', linestyle='--')
plt.plot(kf_interp['x'], kf_interp['y'], label='Kalman Filter', linestyle=':')
plt.plot(ekf_interp['x'], ekf_interp['y'], label='Extended Kalman Filter', linestyle='-.')

plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('Trajectory Comparison')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('trajectory_comparison.png')
plt.show()
