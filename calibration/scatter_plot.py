import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.dates as mdates

# Load CSV
file_path = "MAG_RAW-0.csv"   # update path if needed
df = pd.read_csv(file_path)

# Convert timestamp to datetime
df["timestamp"] = pd.to_datetime(df["timestamp"])

# Convert time to seconds since start (for coloring)
t_seconds = (df["timestamp"] - df["timestamp"].iloc[0]).dt.total_seconds()

# Extract magnetometer data
x = df["mag_x"]
y = df["mag_y"]
z = df["mag_z"]

# Create 3D plot
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection="3d")

sc = ax.scatter(
    x, y, z,
    c=t_seconds,
    cmap="viridis",
    s=5
)

# Labels
ax.set_xlabel("Mag X (µT)")
ax.set_ylabel("Mag Y (µT)")
ax.set_zlabel("Mag Z (µT)")
ax.set_title("3D Magnetometer Trajectory")

# Colorbar
cbar = plt.colorbar(sc)
cbar.set_label("Time (seconds)")

plt.show()