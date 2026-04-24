import numpy as np
import pandas as pd
from Calibration import calibrate_magnetometer, apply_calibration
from Heading_estimate import HeadingEstimator   # <-- your class file


def compute_headings(input_file, output_file):
    cal_df = pd.read_csv("calibration.csv")[["X", "Y", "Z"]].dropna()
    P_sensor = cal_df.to_numpy(dtype=float)

    bias, M, scale_factor, C_total = calibrate_magnetometer(P_sensor)

    # Load Excel data
    df = pd.read_csv("IMU_MAG_ALIGNED.csv")

    # Initialize estimator
    estimator = HeadingEstimator()

    headings = []

    for _, row in df.iterrows():
        gyro = np.array([row['gyr_z'], row['gyr_x'], row['gyr_y']])
        acc  = np.array([row['acc_z'], row['acc_x'], row['acc_y']])
        mag  = np.array([row['mag_x'], row['mag_y'], row['mag_z']])
        
        # mag = apply_calibration(mag, bias, C_total)
        mag = mag[[2, 0, 1]]

        gyro = np.deg2rad(gyro)
        heading = estimator.update(gyro, acc, mag)
        headings.append(heading)

    # Add heading column
    df['Heading_deg'] = headings

    # Save output
    df.to_csv(output_file, index=False)

    print(f"Heading results written to {output_file}")


if __name__ == "__main__":
    compute_headings("IMU_MAG_ALIGNED.csv", "imu_heading_output.csv")
