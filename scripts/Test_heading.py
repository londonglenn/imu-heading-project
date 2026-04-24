import json
import numpy as np
import pandas as pd

from src.Calibration import calibrate_magnetometer, apply_calibration
from src.Heading_estimate import HeadingEstimator


def sensor_to_ned(v: np.ndarray) -> np.ndarray:
    """
    Map sensor-frame vector to NED body frame.

    Sensor frame (given):
        -Z_sensor : forward
        +X_sensor : right
        +Y_sensor : down

    NED body frame:
        X_N : forward (North)
        Y_E : right   (East)
        Z_D : down    (Down)

    Mapping:
        X_N = -Z_sensor
        Y_E =  X_sensor
        Z_D =  Y_sensor
    """
    x_s, y_s, z_s = v
    x_n = -z_s  # forward
    y_e = x_s   # right
    z_d = y_s   # down
    return np.array([x_n, y_e, z_d], dtype=float)


def load_calibration_optional():
    """
    Try to load magnetometer calibration from
    magnetometer_calibration_vectors.json.

    Returns (bias, C_total), or (None, None) if the file is missing
    or invalid, so that the script still runs without calibration.
    """
    try:
        with open("magnetometer_calibration_vectors.json", "r") as f:
            params = json.load(f)
        bias = np.array(params["hard_iron_bias"], dtype=float)
        C_total = np.array(params["overall_calibration_matrix"], dtype=float)

        if bias.shape == (3,) and C_total.shape == (3, 3):
            print("Loaded magnetometer calibration from JSON.")
            return bias, C_total
        else:
            print("Calibration JSON has unexpected shapes; ignoring calibration.")
            return None, None
    except (FileNotFoundError, json.JSONDecodeError, KeyError) as e:
        print(f"Could not load magnetometer calibration JSON (optional): {e}")
        return None, None


def compute_headings(input_file, output_file):
    """
    Offline heading computation that mirrors Live_Heading behavior:

    - Converts gyro/acc/mag from sensor frame to NED body frame.
    - Converts gyro from deg/s to rad/s.
    - Optionally applies magnetometer calibration if available.
    - Feeds NED-frame data into HeadingEstimator and writes Heading_deg.
    """

    # Optional: recompute calibration from calibration.csv if you still want that
    # (kept from your original script, but not required if you rely on JSON).
    # cal_df = pd.read_csv("calibration.csv")[["X", "Y", "Z"]].dropna()
    # P_sensor = cal_df.to_numpy(dtype=float)
    # bias_fit, M, scale_factor, C_fit = calibrate_magnetometer(P_sensor)

    # Load optional calibration (same pattern as Live_Heading)
    bias, C_total = load_calibration_optional()

    # Load aligned IMU+MAG data
    df = pd.read_csv(input_file)

    # Initialize estimator
    estimator = HeadingEstimator()

    headings = []

    for _, row in df.iterrows():
        # 1) Build sensor-frame vectors from CSV
        #    (this matches your existing column usage) [6]
        gyro_sensor = np.array([row["gyr_x"], row["gyr_y"], row["gyr_z"]])
        acc_sensor  = np.array([row["acc_x"], row["acc_y"], row["acc_z"]])
        mag_sensor  = np.array([row["mag_x"], row["mag_y"], row["mag_z"]])

        # 2) Convert to NED body frame (same mapping as Live_Heading) [9]
        gyro_body_deg = sensor_to_ned(gyro_sensor)
        acc_body      = sensor_to_ned(acc_sensor)
        mag_body      = sensor_to_ned(mag_sensor)

        # 3) Optional magnetometer calibration (if JSON was loaded) [9]
        if bias is not None and C_total is not None:
            mag_body = apply_calibration(mag_body, bias, C_total)

        # 4) Convert gyro from deg/s to rad/s before feeding filter [9]
        gyro_body_rad = np.deg2rad(gyro_body_deg)

        # 5) Update heading estimator (expects NED-frame, rad/s gyro) [8]
        heading = estimator.update(gyro_body_rad, acc_body, mag_body)
        headings.append(heading)

    # Add heading column and save
    df["Heading_deg"] = headings
    df.to_csv(output_file, index=False)

    print(f"Heading results written to {output_file}")


if __name__ == "__main__":
    compute_headings("IMU_MAG_ALIGNED.csv", "imu_heading_output.csv")