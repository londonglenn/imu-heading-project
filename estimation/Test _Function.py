import numpy as np
import pandas as pd

from Calibration import calibrate_magnetometer, apply_calibration
from Heading_estimate import HeadingEstimator

import numpy as np

def sensor_to_body(v):
    """Convert ENU-style sensor vector to NED body frame (forward-right-down)."""
    x_s, y_s, z_s = v  # sensor: x=right, y=forward, z=up
    x_b = y_s          # forward
    y_b = x_s          # right
    z_b = -z_s         # down
    return np.array([x_b, y_b, z_b])
# ============================================================
# 1. LOAD CALIBRATION DATA (calibration.csv)
# ============================================================

cal_df = pd.read_csv("calibration.csv")[["X", "Y", "Z"]].dropna()
P_sensor = cal_df.to_numpy(dtype=float)

# Convert each mag sample from sensor frame to body/NED frame
# P_body = np.array([sensor_to_body(v) for v in P_sensor])

print("Calibrating magnetometer...")

bias, M, scale_factor, C_total = calibrate_magnetometer(P_sensor)


print("Calibration complete")
print("Hard-iron bias:", bias)
print("Soft-iron matrix M:\n", M)
print("Scale factor:", scale_factor)
print("Full matrix C_total:\n", C_total)

# ============================================================
# 2. LOAD TEST DATA (test_data.csv)
# ============================================================

test_df = pd.read_csv("test_data.csv")

MAG = test_df[["Mag_X", "Mag_Y", "Mag_Z"]].to_numpy()
ACC = test_df[["Acc_X", "Acc_Y", "Acc_Z"]].to_numpy()
GYR = test_df[["Gyro_X", "Gyro_Y", "Gyro_Z"]].to_numpy()

# Gyro is probably in deg/s → convert to rad/s
# (Remove this if your data is already rad/s)
GYR = np.deg2rad(GYR)

# ============================================================
# 3. Create heading estimator
# ============================================================

estimator = HeadingEstimator(smooth_factor=0.95)

# Output storage
headings = []

print("Processing test data...")

# ============================================================
# 4. PROCESS EACH SAMPLE
# ============================================================

for i in range(len(test_df)):
    mag_raw_sensor = MAG[i]
    acc_sensor     = ACC[i]
    gyro_sensor    = GYR[i]      # and remember: REMOVE the np.deg2rad() line above if units are already rad/s

    # Convert all three sensors to body/NED frame
    # mag_body  = sensor_to_body(mag_raw_sensor)
    # acc_body  = sensor_to_body(acc_sensor)
    # gyro_body = sensor_to_body(gyro_sensor)

    # Apply mag calibration in body frame
    mag_cal_body = apply_calibration(mag_raw_sensor, bias, C_total)

    # Madgwick expects body-frame gyro/acc/mag
    heading = estimator.update( gyro_sensor , acc_sensor, mag_cal_body)

    headings.append(heading)

# Convert to array
headings = np.array(headings)

print("\nDone.")
print("First 10 headings:")
print(headings[:10])

# Save to CSV
out_df = pd.DataFrame({"Heading_deg": headings})
out_df.to_csv("heading_output.csv", index=False)

print("\nHeading data saved to heading_output.csv")
