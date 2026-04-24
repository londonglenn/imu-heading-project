import pandas as pd

# Load data
imu = pd.read_csv("IMU_RAW-1.csv")
mag = pd.read_csv("MAG_RAW-1.csv")

# Parse timestamps
imu["timestamp"] = pd.to_datetime(imu["timestamp"])
mag["timestamp"] = pd.to_datetime(mag["timestamp"])

# Sort
imu = imu.sort_values("timestamp")
mag = mag.sort_values("timestamp")

# Keep only magnetometer data
mag = mag[["timestamp", "mag_x", "mag_y", "mag_z"]]

# --- STEP 1: Set indices ---
imu_idx = imu.set_index("timestamp")
mag_idx = mag.set_index("timestamp")

# --- STEP 2: Union timelines ---
union_index = imu_idx.index.union(mag_idx.index)

# --- STEP 3: Reindex MAG to union timeline ---
mag_reindexed = mag_idx.reindex(union_index)

# --- STEP 4: Interpolate over time ---
mag_interp = (
    mag_reindexed
    .infer_objects(copy=False)
    .interpolate(method="time")
)

# --- STEP 5: Select IMU timestamps ---
mag_at_imu = mag_interp.loc[imu_idx.index]

# --- STEP 6: Combine ---
combined = imu_idx.join(mag_at_imu)

# Save
combined.reset_index().to_csv("IMU_MAG_ALIGNED.csv", index=False)

print("Saved IMU_MAG_ALIGNED.csv")
print("Rows:", len(combined))