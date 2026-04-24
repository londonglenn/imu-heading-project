import numpy as np
from ahrs.filters import Madgwick
from ahrs.common.orientation import q2euler

class HeadingEstimator:
    def __init__(self, smooth_factor=0.95):
        # Madgwick with magnetometer support
        self.madgwick = Madgwick(beta=0.03)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.smooth_factor = smooth_factor
        self.heading_deg = None

    def update(self, gyro, acc, mag):
        
        gyro = np.deg2rad(gyro)

        # USE updateMARG() for gyro+acc+mag
        self.q = self.madgwick.updateMARG(
            q=self.q,
            gyr=gyro,
            acc=acc,
            mag=mag
        )

        # Convert quaternion to yaw/heading
        roll, pitch, yaw = q2euler(self.q)
        heading = np.degrees(yaw) % 360

        # Smooth heading
        if self.heading_deg is None:
            self.heading_deg = heading
        else:
            delta = heading - self.heading_deg
            if delta > 180:  delta -= 360
            if delta < -180: delta += 360

            self.heading_deg += (1 - self.smooth_factor) * delta
            self.heading_deg %= 360

        return self.heading_deg
