import pytest
import numpy as np
try:
    import casadi as ca
    CASADI_AVAILABLE = True
except ImportError:
    CASADI_AVAILABLE = False

from hurodes.joint_mapping import numba_util, casadi_uitl


@pytest.fixture
def solver_params():
    """Fixture providing solver parameters"""
    return {
        "d1": 0.035 / 2.0,
        "d2": 0.035 / 2.0,
        "h1": 0.10,
        "h2": 0.17,
        "r1": 0.04,
        "r2": 0.04,
        "u_x": -0.0445,
        "u_z": 0.00,
    }

def test_euler_to_rotmat():
    """Test with multiple angle combinations"""
    assert CASADI_AVAILABLE, "CasADi is not available"
    
    test_cases = [
        (0.0, 0.0, 0.0),
        (np.pi/4, 0.0, 0.0),
        (0.0, np.pi/4, 0.0),
        (0.0, 0.0, np.pi/4),
        (0.1, 0.2, 0.3),
        (-0.1, -0.2, -0.3),
        (np.pi/6, np.pi/3, np.pi/2),
    ]
    
    for roll, pitch, yaw in test_cases:
        R_casadi = casadi_uitl.euler_to_rotmat(roll, pitch, yaw)
        R_casadi_np = np.array(R_casadi)
        R_numba = numba_util.euler_to_rotmat(roll, pitch, yaw)
        
        np.testing.assert_allclose(R_numba, R_casadi_np, rtol=1e-9, atol=1e-9,
                                 err_msg=f"Failed for roll={roll}, pitch={pitch}, yaw={yaw}")

def test_get_phi(solver_params):
    """Test get_phi with multiple input cases"""
    assert CASADI_AVAILABLE, "CasADi is not available"
    
    test_cases = [
        (np.array([0.01, 0.02, 0.03]), np.array([0.0, 0.05, 0.10]), solver_params["h1"], solver_params["r1"]),
        (np.array([-0.01, 0.02, 0.03]), np.array([0.0, 0.05, 0.10]), solver_params["h1"], solver_params["r1"]),
        (np.array([0.0, 0.02, 0.03]), np.array([0.0, 0.05, 0.10]), solver_params["h1"], solver_params["r1"]),
        (np.array([0.01, -0.02, 0.03]), np.array([0.0, -0.05, 0.10]), solver_params["h2"], solver_params["r2"]),
    ]
    
    for p_u, p_a, h, r in test_cases:
        p_u_ca = ca.vertcat(p_u[0], p_u[1], p_u[2])
        p_a_ca = ca.vertcat(p_a[0], p_a[1], p_a[2])
        phi_casadi = casadi_uitl.get_phi(p_u_ca, p_a_ca, h, r)
        phi_casadi_val = float(phi_casadi)
        
        phi_numba = numba_util.get_phi(p_u, p_a, h, r)
        
        np.testing.assert_allclose(phi_numba, phi_casadi_val, rtol=1e-8, atol=1e-8,
                                 err_msg=f"Failed for p_u={p_u}, p_a={p_a}, h={h}, r={r}")


def test_double_link_inverse(solver_params):
    """Test with multiple angle combinations"""
    assert CASADI_AVAILABLE, "CasADi is not available"
    
    test_cases = [
        (0.0, 0.0),
        (0.1, 0.05),
        (-0.1, -0.05),
        (0.2, 0.1),
        (np.pi/6, np.pi/12),
        (0.5, 0.3),
    ]
    
    for pitch, roll in test_cases:
        phi_l_ca, phi_r_ca = casadi_uitl.double_link_inverse(pitch, roll, **solver_params)
        phi_l_casadi = float(phi_l_ca)
        phi_r_casadi = float(phi_r_ca)
        
        phi_l_numba, phi_r_numba = numba_util.double_link_inverse(
            pitch, roll,
            solver_params["d1"], solver_params["d2"],
            solver_params["h1"], solver_params["h2"],
            solver_params["r1"], solver_params["r2"],
            solver_params["u_x"], solver_params["u_z"]
        )
        
        np.testing.assert_allclose(phi_l_numba, phi_l_casadi, rtol=1e-8, atol=1e-8,
                                 err_msg=f"phi_l failed for pitch={pitch}, roll={roll}")
        np.testing.assert_allclose(phi_r_numba, phi_r_casadi, rtol=1e-8, atol=1e-8,
                                 err_msg=f"phi_r failed for pitch={pitch}, roll={roll}")


def test_compute_jacobian(solver_params):
    """Test jacobian with multiple angle combinations"""
    assert CASADI_AVAILABLE, "CasADi is not available"
    
    test_cases = [
        (0.0, 0.0),
        (0.1, 0.05),
        (-0.1, -0.05),
        (0.2, 0.1),
        (np.pi/6, np.pi/12),
        (0.3, 0.2),
    ]
    
    # Create CasADi function once
    pitch_sym = ca.SX.sym('pitch')
    roll_sym = ca.SX.sym('roll')
    phi_l, phi_r = casadi_uitl.double_link_inverse(pitch_sym, roll_sym, **solver_params)
    jac_ca = ca.jacobian(ca.vertcat(phi_l, phi_r), ca.vertcat(pitch_sym, roll_sym))
    jac_func = ca.Function('jacobian', [pitch_sym, roll_sym], [jac_ca])
    
    for pitch, roll in test_cases:
        jac_casadi = np.array(jac_func(pitch, roll))
        
        jac_numba = numba_util.compute_jacobian(
            pitch, roll,
            solver_params["d1"], solver_params["d2"],
            solver_params["h1"], solver_params["h2"],
            solver_params["r1"], solver_params["r2"],
            solver_params["u_x"], solver_params["u_z"]
        )
        
        np.testing.assert_allclose(jac_numba, jac_casadi, rtol=1e-6, atol=1e-6,
                                 err_msg=f"Failed for pitch={pitch}, roll={roll}")

