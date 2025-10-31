try:
    from numba import jit
    NUMBA_AVAILABLE = True
except ImportError:
    NUMBA_AVAILABLE = False
    jit = lambda x: x  # fallback to no-op decorator
import numpy as np


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
