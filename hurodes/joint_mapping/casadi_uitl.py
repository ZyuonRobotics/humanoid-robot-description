try:
    import casadi as ca
    CASADI_AVAILABLE = True
except ImportError:
    CASADI_AVAILABLE = False
    ca = None

def euler_to_rotmat(roll, pitch, yaw):
    """
    Convert Euler angles (ZYX, yaw-pitch-roll) to rotation matrix
    Input: roll, pitch, yaw (casadi.SX or casadi.MX or float)
    Output: 3x3 rotation matrix (casadi.SX)
    """
    cr = ca.cos(roll)
    sr = ca.sin(roll)
    cp = ca.cos(pitch)
    sp = ca.sin(pitch)
    cy = ca.cos(yaw)
    sy = ca.sin(yaw)

    R = ca.vertcat(
        ca.horzcat(cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr),
        ca.horzcat(sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr),
        ca.horzcat(-sp,   cp*sr,             cp*cr)
    )
    return R

def get_phi(p_u, p_a, h, r):
    d_ly = p_a[1] - p_u[1]
    l_xz = ca.sqrt(h**2 - d_ly**2)

    delta_x = ca.fabs(p_a[0] - p_u[0])
    delta_z = p_a[2] - p_u[2]
    delta_l = ca.sqrt(delta_x**2 + delta_z**2)

    alpha = ca.arctan2(delta_x, delta_z)
    val = (delta_l**2 + r**2 - l_xz**2) / (2 * r * delta_l)
    val = ca.fmax(ca.fmin(val, 1.0), -1.0)  # Clamp val to [-1, 1]
    beta = ca.acos(val)
    phi = alpha + beta - ca.pi / 2

    return -phi * ca.sign(p_u[0])

def double_link_inverse(pitch, roll, d1, d2, h1, h2, r1, r2, u_x, u_z):
    p_lu_3 = ca.vertcat(u_x, +d1, u_z)
    p_ru_3 = ca.vertcat(u_x, -d2, u_z)
    p_la_1 = ca.vertcat(0.0, +d1, h1)
    p_ra_1 = ca.vertcat(0.0, -d2, h2)
    
    p_lu_1 = euler_to_rotmat(roll, pitch, 0) @ p_lu_3
    p_ru_1 = euler_to_rotmat(roll, pitch, 0) @ p_ru_3
    phi_l = get_phi(p_lu_1, p_la_1, h1, r1)
    phi_r = get_phi(p_ru_1, p_ra_1, h2, r2)
    return phi_l, phi_r

def fast_2x2_inverse(A):
    """
    Convert 2x2 matrix to inverse using casadi
    Input: A (casadi.SX or casadi.MX)
    Output: inverse of A (casadi.SX)
    """
    if A.shape != (2, 2):
        raise ValueError("This function is only for 2x2 matrices")
    
    a, b = A[0, 0], A[0, 1]
    c, d = A[1, 0], A[1, 1]
    
    det = a * d - b * c
    
    if abs(det) < 1e-14:
        print("Warning: Matrix is nearly singular, using pseudo-inverse")
        return ca.linalg.pinv(A)
    
    det_inv = 1.0 / det
    return ca.vertcat(
        ca.horzcat(d * det_inv, -b * det_inv), 
        ca.horzcat(-c * det_inv, a * det_inv)
    )
