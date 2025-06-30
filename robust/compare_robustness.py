import pandas as pd
import numpy as np

def compute_rmse(pred_file, odom_file="odom.csv"):
    pred = pd.read_csv(pred_file).drop_duplicates(subset='t')
    odom = pd.read_csv(odom_file).drop_duplicates(subset='t')
    pred_interp = pred.set_index('t').reindex(odom['t'], method='nearest').reset_index()

    rmse_x = np.sqrt(np.mean((pred_interp['x'] - odom['x'])**2))
    rmse_y = np.sqrt(np.mean((pred_interp['y'] - odom['y'])**2))
    return np.sqrt(rmse_x**2 + rmse_y**2)

filter_names = ['kf', 'ekf', 'pf']

print("📊 Robustness Check:")
print("{:<10} {:>10} {:>12}".format("Filter", "Normal [m]", "Bad Init [m]"))
print("-" * 36)

for name in filter_names:
    try:
        normal_rmse = compute_rmse(f"{name}.csv")
        bad_rmse = compute_rmse(f"{name}_bad.csv")
        print(f"{name:<10} {normal_rmse:>10.3f} {bad_rmse:>12.3f}")
    except Exception as e:
        print(f"{name:<10} 🚫 Fehler beim Lesen: {e}")
