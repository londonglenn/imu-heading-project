import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# === Step 1: Load and clean ===
df = pd.read_csv("MAG_RAW-0.csv")
df = df[['X', 'Y', 'Z']].apply(pd.to_numeric, errors='coerce').dropna()
P = df.values

bias = np.array([
    (df['X'].max() + df['X'].min()) / 2,
    (df['Y'].max() + df['Y'].min()) / 2,
    (df['Z'].max() + df['Z'].min()) / 2
])
# === Step 2: Range-centering ===
for col in ['X', 'Y', 'Z']:
    midpoint = (df[col].max() + df[col].min()) / 2
    df[col] = df[col] - midpoint
P = df[['X', 'Y', 'Z']].values
X, Y, Z = P[:, 0], P[:, 1], P[:, 2]

# === Step 3: Algebraic ellipsoid fit ===
D = np.column_stack([
    X*X, Y*Y, Z*Z,
    X*Y, X*Z, Y*Z,
    X, Y, Z,
    np.ones_like(X)
])
_, _, Vt = np.linalg.svd(D)
a = Vt[-1, :]
A, B, C, Dxy, Dxz, Dyz, Gx, Hy, Iz, J = a

# === Step 4: Hard-iron offset (ellipsoid center) ===
A_mat = np.array([
    [2*A, Dxy, Dxz],
    [Dxy, 2*B, Dyz],
    [Dxz, Dyz, 2*C]
])
b_vec = -np.array([Gx, Hy, Iz])
center = np.linalg.solve(A_mat, b_vec)

# === Step 5: Ellipsoid shape matrix ===
Q = np.array([
    [A, Dxy/2, Dxz/2],
    [Dxy/2, B, Dyz/2],
    [Dxz/2, Dyz/2, C]
])
val = center @ Q @ center - J
A_full = Q / val

# === Step 6: Eigen-decomposition (soft-iron) ===
eigvals, eigvecs = np.linalg.eigh(A_full)
eigvals = np.clip(eigvals, 1e-12, None)
radii = 1 / np.sqrt(eigvals)
M = eigvecs @ np.diag(np.sqrt(eigvals)) @ eigvecs.T  # compensation

# === Step 7: Correct the data ===
Ycorr = (M @ (P - center).T).T
r = np.linalg.norm(Ycorr, axis=1)
scale = 1 / np.mean(r)
Ycorr_scaled = Ycorr * scale

# === Step 8: Generate meshes ===
u = np.linspace(0, 2*np.pi, 40)
v = np.linspace(0, np.pi, 20)
# Ellipsoid (before correction)
x_e = radii[0] * np.outer(np.cos(u), np.sin(v))
y_e = radii[1] * np.outer(np.sin(u), np.sin(v))
z_e = radii[2] * np.outer(np.ones_like(u), np.cos(v))
ellipsoid = eigvecs @ np.array([x_e.flatten(), y_e.flatten(), z_e.flatten()])
x_e = ellipsoid[0, :].reshape(x_e.shape) + center[0]
y_e = ellipsoid[1, :].reshape(y_e.shape) + center[1]
z_e = ellipsoid[2, :].reshape(z_e.shape) + center[2]

# Unit sphere (target)
x_s = np.outer(np.cos(u), np.sin(v))
y_s = np.outer(np.sin(u), np.sin(v))
z_s = np.outer(np.ones_like(u), np.cos(v))

# === Step 9: Plot everything ===
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Raw data
ax.scatter(P[:, 0], P[:, 1], P[:, 2], c='b', s=5, alpha=0.4, label='Raw data')

# Ellipsoid
ax.plot_wireframe(x_e, y_e, z_e, color='r', linewidth=0.6, alpha=0.5, label='Fitted ellipsoid')

# Unit sphere
ax.plot_wireframe(x_s, y_s, z_s, color='g', linewidth=0.5, alpha=0.3, label='Unit sphere')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Ellipsoid Fit and Target Unit Sphere (Soft+Hard Iron Calibration)')
ax.legend()
plt.show()

# === Step 10: Diagnostics ===
rms_err = np.sqrt(np.mean((np.linalg.norm(Ycorr_scaled, axis=1) - 1.0)**2))
print("Ellipsoid center (hard-iron offset):", center)
print("Ellipsoid radii (pre-calibration):", radii)
print("Mean radius after correction:", np.mean(np.linalg.norm(Ycorr_scaled, axis=1)))
print("RMS error after correction:", rms_err)

# === Step 11: Compute calibration values for future data ===

# Hard-iron offset: vector to subtract from new raw data
bias = bias +center 
# Soft-iron matrix (already computed as M)
# Scale factor to normalize the sphere
scale_factor = scale

# Full calibration matrix
C_total = scale_factor * M

print("\n=== MAGNETOMETER CALIBRATION PARAMETERS ===")
print("Hard-iron bias (subtract this):", bias)
print("Soft-iron matrix M:\n", M)
print("Global scale factor:", scale_factor)
print("Overall calibration matrix (C = s*M):\n", C_total)

import json

calibration_vectors = {
    "hard_iron_bias": bias.tolist(),
    "soft_iron_matrix_M": M.tolist(),
    "scale_factor": float(scale_factor),
    "overall_calibration_matrix": C_total.tolist()
}

with open("magnetometer_calibration_vectors.json", "w") as f:
    json.dump(calibration_vectors, f, indent=2)

print("\n✅ Calibration vectors saved to magnetometer_calibration_vectors.json")
