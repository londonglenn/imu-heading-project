import pandas as pd

imu = pd.read_csv("IMU_RAW-0.csv")
mag = pd.read_csv("MAG_RAW-0.csv")

imu["timestamp"] = pd.to_datetime(imu["timestamp"])
mag["timestamp"] = pd.to_datetime(mag["timestamp"])

print("IMU time range:")
print(imu["timestamp"].min(), imu["timestamp"].max())

print("\nMAG time range:")
print(mag["timestamp"].min(), mag["timestamp"].max())

print("\nTypical IMU Δt:")
print(imu["timestamp"].diff().median())

print("\nTypical MAG Δt:")
print(mag["timestamp"].diff().median())