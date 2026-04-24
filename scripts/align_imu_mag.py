import argparse
import sys
import pandas as pd


def align_imu_mag(imu_path: str, mag_path: str, output_path: str):
    """
    Align IMU and magnetometer data by interpolating magnetometer samples
    onto the IMU timestamps and saving a combined CSV.

    Expectations:
        - imu_path CSV has a 'timestamp' column (ISO or parseable by pandas)
          and any IMU-related columns (e.g., acc, gyro).
        - mag_path CSV has a 'timestamp' column and magnetometer columns:
              'mag_x', 'mag_y', 'mag_z'.
        - Output will be IMU rows with interpolated mag_x/y/z at the same times.
    """

    # Load data
    try:
        imu = pd.read_csv(imu_path)
        mag = pd.read_csv(mag_path)
    except FileNotFoundError as e:
        print(f"Error: {e}")
        sys.exit(1)

    # Check required columns
    if "timestamp" not in imu.columns:
        print("Error: IMU file is missing 'timestamp' column.")
        sys.exit(1)
    if "timestamp" not in mag.columns:
        print("Error: MAG file is missing 'timestamp' column.")
        sys.exit(1)
    for col in ["mag_x", "mag_y", "mag_z"]:
        if col not in mag.columns:
            print(f"Error: MAG file is missing '{col}' column.")
            sys.exit(1)

    # Parse timestamps
    imu["timestamp"] = pd.to_datetime(imu["timestamp"])
    mag["timestamp"] = pd.to_datetime(mag["timestamp"])

    # Sort by timestamp
    imu = imu.sort_values("timestamp")
    mag = mag.sort_values("timestamp")

    # Keep only magnetometer data columns we care about
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
    combined.reset_index().to_csv(output_path, index=False)

    print(f"Saved {output_path}")
    print("Rows:", len(combined))


def main():
    parser = argparse.ArgumentParser(
        description="Align IMU and MAG CSV files by timestamp and interpolate MAG to IMU times."
    )
    parser.add_argument(
        "--imu",
        default="IMU_RAW-1.csv",
        help="Path to IMU CSV file (default: IMU_RAW-1.csv)",
    )
    parser.add_argument(
        "--mag",
        default="MAG_RAW-1.csv",
        help="Path to MAG CSV file (default: MAG_RAW-1.csv)",
    )
    parser.add_argument(
        "--out",
        default="IMU_MAG_ALIGNED.csv",
        help="Path to output aligned CSV (default: IMU_MAG_ALIGNED.csv)",
    )

    args = parser.parse_args()
    align_imu_mag(args.imu, args.mag, args.out)


if __name__ == "__main__":
    main()