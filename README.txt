# 🧭 IMU + Magnetometer Heading Estimation Pipeline

## Overview

This project estimates heading (yaw) from IMU and magnetometer data.

It supports:

- Offline CSV processing
- Magnetometer calibration
- Heading estimation using a Madgwick filter
- Visualization tools
- Real-time ROS2 heading publishing

---

## Project Structure

```text
.
├── src/
│   ├── Calibration.py
│   ├── Heading_estimate.py
│   └── Live_Heading.py
│
├── scripts/
│   ├── align_imu_mag.py
│   ├── run_cal.py
│   ├── Test_heading.py
│   ├── sanity.py
│   ├── scatter_plot.py
│   └── plot_heading_spiral.py
│
├── data/
├── outputs/
└── README.md
```

---

## Main Files

### `src/Calibration.py`

Contains the magnetometer calibration logic.

It performs ellipsoid-based calibration and returns:

- Hard-iron bias
- Soft-iron correction matrix
- Scale factor
- Final calibration matrix

Calibration is applied as:

```text
v_cal = C_total @ (v_raw - bias)
```

---

### `src/Heading_estimate.py`

Contains the `HeadingEstimator` class.

This class uses the Madgwick filter to fuse:

- Gyroscope
- Accelerometer
- Magnetometer

It outputs heading in degrees from `0` to `360`.

---

### `src/Live_Heading.py`

ROS2 node for real-time heading estimation.

It subscribes to:

```text
/imu/data_raw
/mag/data_raw
```

It publishes:

```text
/heading_deg
/heading_quat
```

---

## Data Flow

```text
Raw IMU CSV + Raw MAG CSV
        |
        v
Timestamp Alignment
        |
        v
IMU_MAG_ALIGNED.csv
        |
        v
Optional Magnetometer Calibration
        |
        v
Madgwick Heading Estimator
        |
        v
imu_heading_output.csv
```

---

## End-to-End Workflow

### 1. Check Raw Data

Run:

```bash
python scripts/sanity.py
```

This checks:

- IMU timestamp range
- MAG timestamp range
- Typical IMU sample interval
- Typical MAG sample interval

---

### 2. Align IMU and Magnetometer Data

Run:

```bash
python scripts/align_imu_mag.py \
  --imu data/IMU_RAW.csv \
  --mag data/MAG_RAW.csv \
  --out outputs/IMU_MAG_ALIGNED.csv
```

This interpolates magnetometer samples onto the IMU timestamps.

Expected input columns:

```text
timestamp
mag_x
mag_y
mag_z
```

The output is a synchronized CSV containing IMU data and interpolated magnetometer data.

---

### 3. Run Magnetometer Calibration

Run:

```bash
python scripts/run_cal.py
```

Expected input:

```text
calibration.csv
```

Expected calibration columns:

```text
X
Y
Z
```

The script outputs:

```text
magnetometer_calibration_vectors.json
```

This file stores:

```text
hard_iron_bias
soft_iron_matrix_M
scale_factor
overall_calibration_matrix
```

---

### 4. Compute Offline Heading

Run:

```bash
python scripts/Test_heading.py
```

Expected input:

```text
IMU_MAG_ALIGNED.csv
```

Output:

```text
imu_heading_output.csv
```

The offline heading pipeline:

1. Loads aligned IMU + MAG data
2. Converts sensor-frame vectors into NED body frame
3. Applies optional magnetometer calibration
4. Converts gyro from deg/s to rad/s
5. Updates the heading estimator
6. Saves heading output as `Heading_deg`

---

### 5. Plot Heading

Run:

```bash
python scripts/plot_heading_spiral.py
```

Expected input:

```text
heading_output.csv
```

This creates a polar spiral plot of heading over time.

---

### 6. Plot Magnetometer Data

Run:

```bash
python scripts/scatter_plot.py
```

Expected input:

```text
MAG_RAW-0.csv
```

This creates a 3D magnetometer scatter plot.

Use this to inspect whether the magnetometer data looks like:

- An ellipsoid before calibration
- A sphere after calibration

---

## Coordinate Frames

The sensor frame is assumed to be:

```text
-Z_sensor : forward
+X_sensor : right
+Y_sensor : down
```

The desired body frame is NED:

```text
X_N : forward / north
Y_E : right / east
Z_D : down
```

The conversion is:

```text
X_N = -Z_sensor
Y_E =  X_sensor
Z_D =  Y_sensor
```

This mapping is used in both:

- Offline heading estimation
- ROS2 live heading estimation

---

## Gyroscope Units

The gyroscope is assumed to be in:

```text
deg/s
```

Before sensor fusion, it is converted to:

```text
rad/s
```

using:

```text
gyro_rad = np.deg2rad(gyro_deg)
```

---

## Magnetometer Calibration

The calibration process corrects for:

- Hard-iron distortion
- Soft-iron distortion

The final corrected vector is:

```text
v_cal = C_total @ (v_raw - bias)
```

Where:

```text
bias    = hard-iron offset
C_total = total calibration matrix
```

If the calibration JSON file is missing, the offline and live pipelines can still run, but heading quality may be worse.

---

## Heading Estimation

Heading is estimated using a Madgwick MARG filter.

Inputs:

```text
gyro : angular velocity, rad/s
acc  : acceleration vector
mag  : magnetic field vector
```

Output:

```text
heading_deg : heading/yaw in degrees from 0 to 360
```

The heading output is smoothed using an exponential smoothing factor.

A higher smoothing factor gives smoother but slower heading response.

---

## ROS2 Live Heading

Run:

```bash
python src/Live_Heading.py
```

Default subscribed topics:

```text
/imu/data_raw
/mag/data_raw
```

Default published topics:

```text
/heading_deg
/heading_quat
```

The ROS2 node:

1. Reads IMU data
2. Reads magnetometer data
3. Converts vectors from sensor frame to NED
4. Converts gyro from deg/s to rad/s
5. Applies magnetometer calibration if available
6. Normalizes accelerometer and magnetometer vectors
7. Runs the heading estimator
8. Publishes heading and quaternion

---

## Dependencies

Install Python dependencies:

```bash
pip install numpy pandas matplotlib ahrs
```

For ROS2, install the required ROS2 Python packages for your distro.

Example:

```bash
sudo apt install ros-<distro>-rclpy
```

Replace `<distro>` with your ROS2 distribution, such as:

```text
humble
iron
jazzy
```

---

## Expected CSV Columns

### IMU CSV

Expected columns include:

```text
timestamp
acc_x
acc_y
acc_z
gyr_x
gyr_y
gyr_z
```

### Magnetometer CSV

Expected columns include:

```text
timestamp
mag_x
mag_y
mag_z
```

### Calibration CSV

Expected columns:

```text
X
Y
Z
```

---

## Output Files

| File | Description |
|---|---|
| `IMU_MAG_ALIGNED.csv` | Time-aligned IMU and magnetometer data |
| `magnetometer_calibration_vectors.json` | Saved calibration parameters |
| `imu_heading_output.csv` | Offline heading output |
| `calibrated_output.csv` | Optional calibrated magnetometer output |

---

## Quick Start

```bash
python scripts/sanity.py
python scripts/align_imu_mag.py --imu data/IMU_RAW.csv --mag data/MAG_RAW.csv --out outputs/IMU_MAG_ALIGNED.csv
python scripts/run_cal.py
python scripts/Test_heading.py
python scripts/plot_heading_spiral.py
```

---

## Debugging

### Heading looks wrong

Check:

- Axis mapping
- Gyro units
- Magnetometer calibration
- Magnetic interference nearby

---

### Heading is noisy

Try:

- Increasing the smoothing factor
- Checking raw magnetometer stability
- Checking accelerometer noise

---

### Heading drifts

Try:

- Re-running magnetometer calibration
- Moving away from magnetic interference
- Checking whether calibration JSON is being loaded
- Verifying that the sensor frame matches the assumed frame

---

### Calibration looks bad

Check:

- Calibration motion covered full 3D rotation
- Data was not collected near magnetic interference
- Magnetometer samples are not saturated
- Input axes are consistent

---

## Handoff Notes

- `src/` contains reusable project logic.
- `scripts/` contains executable utilities and workflow scripts.
- Keep coordinate-frame conventions consistent.
- Calibration is optional but strongly recommended.
- Offline and ROS2 pipelines are intended to use the same assumptions.
- Do not change axis mapping unless the sensor mounting orientation changes.

---