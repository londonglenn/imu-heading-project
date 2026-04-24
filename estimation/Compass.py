import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv("heading_output.csv")
theta = np.deg2rad(df["Heading_deg"])
r = np.linspace(0, 1, len(theta))

plt.figure(figsize=(7,7))
ax = plt.subplot(111, polar=True)
ax.plot(theta, r, linewidth=1)

ax.set_theta_zero_location("N")
ax.set_theta_direction(-1)

plt.title("Heading Over Time (Spiral View)")
plt.show()
