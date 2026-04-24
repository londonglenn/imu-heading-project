import numpy as np
from ahrs.filters import Madgwick
from ahrs.common.orientation import q2euler


class HeadingEstimator:
    """
    Estimate yaw/heading from IMU + magnetometer using the Madgwick filter.

    Assumptions:
        - gyro: np.ndarray shape (3,), angular velocity in rad/s (body frame)
        - acc:  np.ndarray shape (3,), linear acceleration (body frame)
        - mag:  np.ndarray shape (3,), magnetic field (body frame)
        - All vectors are already in the desired body frame; no axis
          remapping or unit conversion is done inside this class.

    The heading returned is yaw in degrees in [0, 360).
    """

    def __init__(self, smooth_factor: float = 0.95):
        # Madgwick with magnetometer support (MARG)
        self.madgwick = Madgwick(beta=0.03)
        # Current orientation quaternion (w, x, y, z)
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

        # Exponential smoothing factor for heading:
        #   smooth_factor close to 1.0 -> heavier smoothing
        self.smooth_factor = float(smooth_factor)

        # Last smoothed heading in degrees, or None if not initialized
        self.heading_deg = None

    def update(self, gyro: np.ndarray, acc: np.ndarray, mag: np.ndarray) -> float:
        """
        Update the filter with a new gyro/acc/mag sample and return the
        smoothed heading in degrees.

        Parameters
        ----------
        gyro : np.ndarray, shape (3,)
            Angular velocity in rad/s, body frame.
        acc : np.ndarray, shape (3,)
            Linear acceleration vector, body frame.
        mag : np.ndarray, shape (3,)
            Magnetic field vector, body frame.

        Returns
        -------
        float
            Smoothed heading in degrees in [0, 360).
        """

        # NOTE: No unit conversion or axis remapping here.
        # Callers must ensure gyro is in rad/s and vectors are in the
        # correct body frame before calling this method.

        # Use updateMARG() for gyro + acc + mag fusion
        self.q = self.madgwick.updateMARG(
            q=self.q,
            gyr=gyro,
            acc=acc,
            mag=mag,
        )

        # Convert quaternion to yaw/heading
        roll, pitch, yaw = q2euler(self.q)
        heading = np.degrees(yaw) % 360.0

        # Exponential smoothing of heading with proper wrap-around
        if self.heading_deg is None:
            self.heading_deg = heading
        else:
            delta = heading - self.heading_deg

            # Wrap delta into [-180, 180] to avoid jumps across 0/360
            if delta > 180.0:
                delta -= 360.0
            if delta < -180.0:
                delta += 360.0

            # Apply smoothing
            alpha = 1.0 - self.smooth_factor
            self.heading_deg = (self.heading_deg + alpha * delta) % 360.0

        return float(self.heading_deg)