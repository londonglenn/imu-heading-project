# calibration/run_cal.py
import json
import numpy as np
import pandas as pd
from Calibration import calibrate_magnetometer, apply_calibration

def main():
    # 1. Load calibration data
    calib_df = pd.read_csv("calibration.csv")      # in calibration/
    P = calib_df[["X", "Y", "Z"]].values.astype(float)

    # 2. Run ellipsoid calibration
    bias, M, scale_factor, C_total = calibrate_magnetometer(P)

    # 3. Save parameters for later use
    params = {
        "hard_iron_bias": bias.tolist(),
        "soft_iron_matrix_M": M.tolist(),
        "scale_factor": float(scale_factor),
        "overall_calibration_matrix": C_total.tolist(),
    }
    with open("magnetometer_calibration_vectors.json", "w") as f:
        json.dump(params, f, indent=2)

    # 4. (optional) apply to a test file as an example
    #    and save to calibrated_output.csv
    if "Mag_X" in calib_df.columns:
        raw_test = calib_df[["Mag_X", "Mag_Y", "Mag_Z"]].values.astype(float)
        calibrated = np.array([
            apply_calibration(v, bias, C_total) for v in raw_test
        ])
        pd.DataFrame(calibrated,
                     columns=["X_cal", "Y_cal", "Z_cal"]
                     ).to_csv("calibrated_output.csv", index=False)

    print("Calibration complete")
    print("Hard-iron bias:", bias)
    print("Scale factor:", scale_factor)

if __name__ == "__main__":
    main()