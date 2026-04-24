Here is a **single, clean README.md** you can copy and paste directly:

---

```markdown
# 🧭 IMU + Magnetometer Heading Estimation Pipeline

## Overview
This project implements a complete pipeline for estimating heading (yaw) using IMU and magnetometer data.

It supports:
- Offline processing (CSV-based)
- Real-time heading estimation (ROS2)
- Magnetometer calibration
- Visualization and debugging tools

---

## 📁 Project Structure

```

.
├── src/
│   ├── Calibration.py          # Magnetometer calibration (ellipsoid fit)
│   ├── Heading_estimate.py     # Madgwick-based heading estimator
│   └── Live_Heading.py         # ROS2 real-time node
│
├── scripts/
│   ├── align_imu_mag.py        # Align IMU + MAG timestamps
│   ├── run_cal.py              # Run calibration and save parameters
│   ├── Test_heading.py         # Offline heading computation
│   ├── sanity.py               # Data sanity checks
│   ├── scatter_plot.py         # 3D magnetometer visualization
│   └── plot_heading_spiral.py  # Heading visualization
│
├── data/                       # Raw input CSV files
├── outputs/                    # Generated results
└── README.md

````

---

## 🔄 End-to-End Workflow

### 1. Sanity Check Data

```bash
python scripts/sanity.py
````

* Confirms timestamp ranges
* Checks sampling intervals

---

### 2. Align IMU + Magnetometer Data

```bash
python scripts/align_imu_mag.py \
  --imu data/IMU_RAW.csv \
  --mag data/MAG_RAW.csv \
  --out outputs/IMU_MAG_ALIGNED.csv
```

* Interpolates magnetometer data to IMU timestamps
* Produces synchronized dataset

---

### 3. Magnetometer Calibration

```bash
python scripts/run_cal.py
```

* Computes:

  * Hard-iron bias
  * Soft-iron correction
* Outputs:

```
outputs/magnetometer_calibration_vectors.json
```

---

### 4. Compute Heading (Offline)

```bash
python scripts/Test_heading.py
```

Output:

```
outputs/imu_heading_output.csv
```

Pipeline:

* Convert sensor frame → NED frame
* Apply calibration (if available)
* Convert gyro (deg/s → rad/s)
* Run Madgwick filter
* Smooth heading output

---

### 5. Visualization

#### Spiral Plot (Heading vs Time)

```bash
python scripts/plot_heading_spiral.py
```

#### 3D Magnetometer Plot

```bash
python scripts/scatter_plot.py
```

* Before calibration → ellipsoid
* After calibration → sphere

---

## 🧠 Core Concepts

### Sensor Frame → NED Frame

Sensor orientation:

```
-Z_sensor : forward
+X_sensor : right
+Y_sensor : down
```

Converted to NED:

```
X (North) = -Z_sensor
Y (East)  =  X_sensor
Z (Down)  =  Y_sensor
```

---

### Heading Estimation

Implemented in:

```
src/Heading_estimate.py
```

Uses:

* Madgwick MARG filter
* Gyro + accel + magnetometer fusion

Outputs:

* Quaternion orientation
* Heading (yaw in degrees)

---

### Magnetometer Calibration Model

```
v_cal = C_total @ (v_raw - bias)
```

Where:

* `bias` = hard-iron offset
* `C_total` = soft-iron + scale correction

---

## ⚙️ Real-Time (ROS2)

Run:

```bash
python src/Live_Heading.py
```

Subscribes to:

* `/imu/data_raw`
* `/mag/data_raw`

Publishes:

* `/heading_deg`
* `/heading_quat`

Features:

* Optional calibration loading (JSON)
* Frame conversion
* Real-time sensor fusion

---

## 📦 Dependencies

```bash
pip install numpy pandas matplotlib ahrs
```

For ROS2:

```bash
sudo apt install ros-<distro>-rclpy
```

---

## ⚠️ Assumptions

* Gyroscope input is in **deg/s**
* Magnetometer columns:

```
mag_x, mag_y, mag_z
```

* Timestamps must be parseable by pandas
* Calibration requires full 3D motion (not planar)

---

## 🧪 Debugging Tips

### Incorrect Heading

* Check calibration (should form a sphere)
* Verify axis mapping
* Confirm gyro units

### Noisy Output

* Increase smoothing factor in `HeadingEstimator`

### Drift

* Recalibrate magnetometer
* Check for magnetic interference

---

## 🧾 Outputs

| File                                  | Description              |
| ------------------------------------- | ------------------------ |
| IMU_MAG_ALIGNED.csv                   | Time-aligned sensor data |
| imu_heading_output.csv                | Final heading output     |
| magnetometer_calibration_vectors.json | Calibration parameters   |
| calibrated_output.csv                 | Optional debug output    |

---

## 🚀 Quick Start

```bash
python scripts/sanity.py
python scripts/align_imu_mag.py
python scripts/run_cal.py
python scripts/Test_heading.py
python scripts/plot_heading_spiral.py
```

---

## 🤝 Handoff Notes

* `src/` contains reusable core logic
* `scripts/` contains execution and utilities
* Calibration is optional but strongly recommended
* Frame conventions must remain consistent across all modules
* Offline and ROS pipelines are designed to match behavior

---

```

---

If you want, I can tighten this further into:
- a **super minimal operator README (5 commands only)**  
- or a **deep engineering doc with diagrams + math explanation**

Just tell me which direction.
```
