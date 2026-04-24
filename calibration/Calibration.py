import numpy as np

def calibrate_magnetometer(P):
    """
    P : Nx3 numpy array of raw magnetometer samples
        columns = X, Y, Z

    Returns:
        bias          = hard-iron offset (vector to subtract)
        M             = soft-iron correction matrix (3×3)
        scale_factor  = average radius scaler
        C_total       = final calibration matrix (s * M)
    """

    # === Step 1: Range-center hard-iron shift (rough) ===
    dfX, dfY, dfZ = P[:,0], P[:,1], P[:,2]
    df = {"X": dfX.copy(), "Y": dfY.copy(), "Z": dfZ.copy()}

    # Range-centering
    for i, col in enumerate(["X", "Y", "Z"]):
        mid = (P[:,i].max() + P[:,i].min()) / 2
        P[:,i] -= mid

    # === Step 2: Ellipsoid fit ===
    X, Y, Z = P[:,0], P[:,1], P[:,2]

    D = np.column_stack([
        X*X, Y*Y, Z*Z,
        X*Y, X*Z, Y*Z,
        X, Y, Z,
        np.ones_like(X)
    ])
    _, _, Vt = np.linalg.svd(D)
    a = Vt[-1, :]

    A, B, C, Dxy, Dxz, Dyz, Gx, Hy, Iz, J = a

    # === Step 3: Ellipsoid center = hard-iron bias ===
    A_mat = np.array([
        [2*A, Dxy, Dxz],
        [Dxy, 2*B, Dyz],
        [Dxz, Dyz, 2*C]
    ])
    b_vec = -np.array([Gx, Hy, Iz])
    center = np.linalg.solve(A_mat, b_vec)

    # === Step 4: Ellipsoid quadratic form (soft iron) ===
    Q = np.array([
        [A, Dxy/2, Dxz/2],
        [Dxy/2, B, Dyz/2],
        [Dxz/2, Dyz/2, C]
    ])
    val = center @ Q @ center - J
    A_full = Q / val

    # === Step 5: Eigen-decomposition → axes and radii ===
    eigvals, eigvecs = np.linalg.eigh(A_full)
    eigvals = np.clip(eigvals, 1e-12, None)

    radii = 1.0 / np.sqrt(eigvals)

    # Soft-iron correction matrix
    M = eigvecs @ np.diag(np.sqrt(eigvals)) @ eigvecs.T

    # === Step 6: Apply correction to compute global scale factor ===
    Ycorr = (M @ (P - center).T).T
    r = np.linalg.norm(Ycorr, axis=1)
    scale_factor = 1.0 / np.mean(r)

    # === Final calibration matrix ===
    C_total = scale_factor * M

    return center, M, scale_factor, C_total

def apply_calibration(v_raw, bias, C_total):
    return C_total @ (v_raw - bias)
