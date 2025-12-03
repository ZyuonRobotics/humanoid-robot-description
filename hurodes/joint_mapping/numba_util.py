import numpy as np
try:
    from numba import jit
    NUMBA_AVAILABLE = True
except ImportError:
    NUMBA_AVAILABLE = False
    def jit(func=None, **kwargs):
        if func is None:
            return lambda f: f
        return func


@jit(nopython=True, fastmath=True, cache=True)
def euler_to_rotmat(roll, pitch, yaw):
    """
    JIT version of euler_to_rotmat function.
    Convert Euler angles (ZYX, yaw-pitch-roll) to rotation matrix
    Input: roll, pitch, yaw (float)
    Output: 3x3 rotation matrix (numpy array)
    """
    cr = np.cos(roll)
    sr = np.sin(roll)
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cy = np.cos(yaw)
    sy = np.sin(yaw)

    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,             cp*cr]
    ])
    return R

@jit(nopython=True, fastmath=True, cache=True)
def get_phi(p_u, p_a, h, r):
    """
    JIT version of get_phi using numpy instead of casadi
    Input: p_u, p_a (numpy arrays), h, r (float)
    Output: phi (float)
    """
    d_ly = p_a[1] - p_u[1]
    l_xz = np.sqrt(h**2 - d_ly**2)

    delta_x = np.abs(p_a[0] - p_u[0])
    delta_z = p_a[2] - p_u[2]
    delta_l = np.sqrt(delta_x**2 + delta_z**2)

    alpha = np.arctan2(delta_x, delta_z)
    val = (delta_l**2 + r**2 - l_xz**2) / (2 * r * delta_l)
    val = max(min(val, 1.0), -1.0)  # Clamp val to [-1, 1]
    beta = np.arccos(val)
    phi = alpha + beta - np.pi / 2

    return -phi * np.sign(p_u[0])

@jit(nopython=True, fastmath=True, cache=True)
def double_link_inverse(pitch, roll, d1, d2, h1, h2, r1, r2, u_x, u_z):
    """
    JIT version of inverse function using numpy instead of casadi
    Input: pitch, roll (float), and solver parameters as separate floats
    Output: phi_l, phi_r (float, float)
    """
    p_lu_3 = np.array([u_x, d1, u_z])
    p_ru_3 = np.array([u_x, -d2, u_z])
    p_la_1 = np.array([0.0, d1, h1])
    p_ra_1 = np.array([0.0, -d2, h2])

    R = euler_to_rotmat(roll, pitch, 0.0)
    p_lu_1 = R @ p_lu_3
    p_ru_1 = R @ p_ru_3

    phi_l = get_phi(p_lu_1, p_la_1, h1, r1)
    phi_r = get_phi(p_ru_1, p_ra_1, h2, r2)

    return phi_l, phi_r

@jit(nopython=True, fastmath=True, cache=True)
def fast_2x2_inverse(A):
    """
    JIT version of fast_2x2_inverse function.
    Input: A (numpy array)
    Output: inverse of A (numpy array)
    """
    if A.shape != (2, 2):
        raise ValueError("This function is only for 2x2 matrices")
    
    a, b = A[0, 0], A[0, 1]
    c, d = A[1, 0], A[1, 1]
    
    det = a * d - b * c
    
    if abs(det) < 1e-14:
        print("Warning: Matrix is nearly singular, using pseudo-inverse")
        return np.linalg.pinv(A)
    
    det_inv = 1.0 / det
    return np.array([[d * det_inv, -b * det_inv],
                     [-c * det_inv, a * det_inv]])


@jit(nopython=True, fastmath=True, cache=True)
def get_phi_gradient(x_u, y_u, z_u, y_a, z_a, h, r):
    """
    Computes gradient of phi w.r.t P_u (x_u, y_u, z_u)
    Assuming x_a = 0.
    
    Args:
        x_u, y_u, z_u: Upper attachment point coordinates (after rotation)
        y_a: Y-coordinate of actuator attachment point
        z_a: Z-coordinate of actuator attachment point
        h: Link length
        r: Crank radius
    
    Returns:
        Gradient array [d_phi/d_x, d_phi/d_y, d_phi/d_z]
    """
    # Forward pass reconstruction to get intermediate values
    d_ly = y_a - y_u
    l_xz = np.sqrt(h**2 - d_ly**2)
    
    delta_x = np.abs(x_u)
    delta_z = z_a - z_u
    delta_l = np.sqrt(delta_x**2 + delta_z**2)
    
    # Prevent division by zero
    delta_l = max(delta_l, 1e-7)
    l_xz = max(l_xz, 1e-7)
    
    val = (delta_l**2 + r**2 - l_xz**2) / (2 * r * delta_l)
    
    # Clamp val for acos
    val_clamped = max(min(val, 1.0 - 1e-7), -1.0 + 1e-7)
    
    # Derivatives
    # d_beta/d_val = -1 / sqrt(1 - val^2)
    d_beta_d_val = -1.0 / np.sqrt(1.0 - val_clamped**2)
    
    # Zero gradient if val was out of bounds (mimic autograd behavior approximately)
    mask = 1.0 if (val >= -1.0) and (val <= 1.0) else 0.0
    d_beta_d_val = d_beta_d_val * mask
    
    # 1. Derivatives of alpha w.r.t (delta_x, delta_z)
    d_alpha_d_dx = delta_z / (delta_l**2)
    d_alpha_d_dz = -delta_x / (delta_l**2)
    
    # 2. Derivatives of val w.r.t (delta_l, l_xz)
    d_val_d_dl = (delta_l**2 - r**2 + l_xz**2) / (2 * r * delta_l**2)
    d_val_d_lxz = -l_xz / (r * delta_l)
    
    # 3. Chain rule back to x_u, y_u, z_u
    
    # w.r.t y_u
    d_lxz_d_yu = d_ly / l_xz
    d_phi_raw_d_yu = d_beta_d_val * d_val_d_lxz * d_lxz_d_yu
    
    # w.r.t x_u
    sign_x = np.sign(x_u) if x_u != 0 else 1.0
    d_dl_d_xu = (delta_x / delta_l) * sign_x
    d_alpha_d_xu = d_alpha_d_dx * sign_x
    d_phi_raw_d_xu = d_alpha_d_xu + d_beta_d_val * d_val_d_dl * d_dl_d_xu
    
    # w.r.t z_u
    d_dl_d_zu = -delta_z / delta_l
    d_alpha_d_zu = -d_alpha_d_dz
    d_phi_raw_d_zu = d_alpha_d_zu + d_beta_d_val * d_val_d_dl * d_dl_d_zu
    
    # Final gradient: d_phi/d_Pu = -sign(x_u) * d_phi_raw/d_Pu
    sx = sign_x
    grad_x = -sx * d_phi_raw_d_xu
    grad_y = -sx * d_phi_raw_d_yu
    grad_z = -sx * d_phi_raw_d_zu
    
    return np.array([grad_x, grad_y, grad_z])


@jit(nopython=True, fastmath=True, cache=True)
def compute_jacobian(pitch, roll, d1, d2, h1, h2, r1, r2, u_x, u_z):
    """
    Compute Jacobian matrix d(motor_pos)/d(joint_pos) analytically for a single sample.
    
    Args:
        pitch, roll: Joint positions (scalars)
        d1, d2: Half distances between linkages
        h1, h2: Link lengths
        r1, r2: Crank radii
        u_x, u_z: Upper attachment point offsets
    
    Returns:
        Jacobian matrix, shape (2, 2)
    """
    # Precompute trig functions
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cr = np.cos(roll)
    sr = np.sin(roll)
    
    # --- LEFT LEG ---
    # p_lu_3 = [u_x, d1, u_z]
    x3_l, y3_l, z3_l = u_x, d1, u_z
    
    # Position p_lu_1 = R * p_lu_3
    x_u_l = cp*x3_l + sp*sr*y3_l + sp*cr*z3_l
    y_u_l = cr*y3_l - sr*z3_l
    z_u_l = -sp*x3_l + cp*sr*y3_l + cp*cr*z3_l
    
    # Derivatives of p_lu_1 w.r.t pitch (theta_p)
    dx_dp_l = -sp*x3_l + cp*sr*y3_l + cp*cr*z3_l
    dy_dp_l = 0.0
    dz_dp_l = -cp*x3_l - sp*sr*y3_l - sp*cr*z3_l
    
    # Derivatives of p_lu_1 w.r.t roll (theta_r)
    dx_dr_l = sp*cr*y3_l - sp*sr*z3_l
    dy_dr_l = -sr*y3_l - cr*z3_l
    dz_dr_l = cp*cr*y3_l - cp*sr*z3_l
    
    # Gradient of phi_l w.r.t position
    grad_phi_pu_l = get_phi_gradient(x_u_l, y_u_l, z_u_l, d1, h1, h1, r1)
    
    # Chain rule for Left
    J_00 = grad_phi_pu_l[0] * dx_dp_l + grad_phi_pu_l[1] * dy_dp_l + grad_phi_pu_l[2] * dz_dp_l
    J_01 = grad_phi_pu_l[0] * dx_dr_l + grad_phi_pu_l[1] * dy_dr_l + grad_phi_pu_l[2] * dz_dr_l
    
    # --- RIGHT LEG ---
    # p_ru_3 = [u_x, -d2, u_z]
    x3_r, y3_r, z3_r = u_x, -d2, u_z
    
    x_u_r = cp*x3_r + sp*sr*y3_r + sp*cr*z3_r
    y_u_r = cr*y3_r - sr*z3_r
    z_u_r = -sp*x3_r + cp*sr*y3_r + cp*cr*z3_r
    
    # Derivatives w.r.t pitch
    dx_dp_r = -sp*x3_r + cp*sr*y3_r + cp*cr*z3_r
    dy_dp_r = 0.0
    dz_dp_r = -cp*x3_r - sp*sr*y3_r - sp*cr*z3_r
    
    # Derivatives w.r.t roll
    dx_dr_r = sp*cr*y3_r - sp*sr*z3_r
    dy_dr_r = -sr*y3_r - cr*z3_r
    dz_dr_r = cp*cr*y3_r - cp*sr*z3_r
    
    # Gradient of phi_r
    grad_phi_pu_r = get_phi_gradient(x_u_r, y_u_r, z_u_r, -d2, h2, h2, r2)
    
    # Chain rule for Right
    J_10 = grad_phi_pu_r[0] * dx_dp_r + grad_phi_pu_r[1] * dy_dp_r + grad_phi_pu_r[2] * dz_dp_r
    J_11 = grad_phi_pu_r[0] * dx_dr_r + grad_phi_pu_r[1] * dy_dr_r + grad_phi_pu_r[2] * dz_dr_r
    
    # Assemble Jacobian [2, 2]
    J = np.array([[J_00, J_01],
                  [J_10, J_11]])
    
    return J
