import numpy as np

def calibrate_magnetometer(P: np.ndarray):
    """
    Perform ellipsoid-based magnetometer calibration.

    Parameters
    ----------
    P : np.ndarray, shape (N, 3)
        Raw magnetometer samples in a consistent frame
        (e.g. body frame X,Y,Z). No axis remapping or
        unit conversion is performed inside this function.

    Returns
    -------
    bias : np.ndarray, shape (3,)
        Hard-iron offset (vector to subtract from raw data).
    M : np.ndarray, shape (3, 3)
        Soft-iron correction matrix.
    scale_factor : float
        Global scale factor used to normalize the corrected
        data to a unit sphere.
    C_total : np.ndarray, shape (3, 3)
        Overall calibration matrix C = scale_factor * M.
        A calibrated sample v_cal is computed as:
            v_cal = C_total @ (v_raw - bias)
    """

    # === Step 1: Rough range-centering (approximate hard-iron) ===
    # Shift each axis so that the mid-point of its min/max is at zero.
    for i, col in enumerate(["X", "Y", "Z"]):
        mid = (P[:, i].max() + P[:, i].min()) / 2.0
        P[:, i] -= mid

    # === Step 2: Algebraic ellipsoid fit ===
    X, Y, Z = P[:, 0], P[:, 1], P[:, 2]

    D = np.column_stack([
        X*X, Y*Y, Z*Z,
        X*Y, X*Z, Y*Z,
        X, Y, Z,
        np.ones_like(X),
    ])

    # Solve D * a = 0 via SVD (last row of V^T)
    _, _, Vt = np.linalg.svd(D)
    a = Vt[-1, :]

    A, B, C, Dxy, Dxz, Dyz, Gx, Hy, Iz, J = a

    # === Step 3: Ellipsoid center (hard-iron bias) ===
    A_mat = np.array([
        [2*A,    Dxy,   Dxz],
        [Dxy,   2*B,   Dyz],
        [Dxz,   Dyz,   2*C],
    ])
    b_vec = -np.array([Gx, Hy, Iz])
    center = np.linalg.solve(A_mat, b_vec)  # this is the hard-iron bias

    # === Step 4: Ellipsoid quadratic form (soft-iron) ===
    Q = np.array([
        [A,       Dxy/2.0, Dxz/2.0],
        [Dxy/2.0, B,       Dyz/2.0],
        [Dxz/2.0, Dyz/2.0, C      ],
    ])

    # Normalize so that the ellipsoid equation becomes x^T A_full x = 1
    val = center @ Q @ center - J
    A_full = Q / val

    # === Step 5: Eigen-decomposition → axes and radii ===
    eigvals, eigvecs = np.linalg.eigh(A_full)
    eigvals = np.clip(eigvals, 1e-12, None)  # avoid division by zero
    radii = 1.0 / np.sqrt(eigvals)

    # Soft-iron correction matrix M
    # (Maps ellipsoid into a sphere up to a global scale)
    M = eigvecs @ np.diag(np.sqrt(eigvals)) @ eigvecs.T

    # === Step 6: Compute global scale factor ===
    # Apply M and bias to see how far points are from unit radius
    Ycorr = (M @ (P - center).T).T
    r = np.linalg.norm(Ycorr, axis=1)
    scale_factor = 1.0 / np.mean(r)

    # === Final calibration matrix ===
    C_total = scale_factor * M

    bias = center
    return bias, M, scale_factor, C_total


def apply_calibration(v_raw: np.ndarray,
                      bias: np.ndarray,
                      C_total: np.ndarray) -> np.ndarray:
    """
    Apply magnetometer calibration to a single sample.

    Parameters
    ----------
    v_raw : np.ndarray, shape (3,)
        Raw magnetometer sample in the same frame used
        for calibration.
    bias : np.ndarray, shape (3,)
        Hard-iron offset returned by calibrate_magnetometer.
    C_total : np.ndarray, shape (3, 3)
        Overall calibration matrix returned by
        calibrate_magnetometer.

    Returns
    -------
    np.ndarray, shape (3,)
        Calibrated magnetometer sample.
    """
    return C_total @ (v_raw - bias)