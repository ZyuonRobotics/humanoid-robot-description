import numpy as np
import time
try:
    from numba import jit
    NUMBA_AVAILABLE = True
except ImportError:
    NUMBA_AVAILABLE = False
    def jit(func=None, **kwargs):
        if func is None:
            return lambda f: f
        return func

try:
    import casadi as ca
    CASADI_AVAILABLE = True
except ImportError:
    CASADI_AVAILABLE = False
    ca = None

from hurodes.joint_mapping.base_solver import BaseSolver
from hurodes.joint_mapping import numba_util, casadi_uitl

# @jit(nopython=True, fastmath=True, cache=True)
def solve_ik_numba(initial_pos, q_m, d1, d2, h1, h2, r1, r2, u_x, u_z, max_iter=100, tol=1e-4, alpha=0.9):
    """
    Solve inverse problem using Jacobian matrix iteration
    Parameters:
        q_j0: Initial joint variables (array shaped like [q1, q2])
        q_m: Target motor angles (array shaped like [phi_l, phi_r])
        d1, d2, h1, h2, r1, r2, u_x, u_z: Solver parameters
        max_iter: Maximum number of iterations
        tol: Error tolerance
        alpha: Step size parameter (damping factor to avoid oscillation)

    Returns:
        q_j: Optimized joint variables
        err: Final error
        iter_count: Actual number of iterations
    """
    q_j = initial_pos.copy()  # Avoid modifying original data
    phi_l0, phi_r0 = numba_util.double_link_inverse(float(q_j[0]), float(q_j[1]), d1, d2, h1, h2, r1, r2, u_x, u_z)
    q_m0 = np.array([float(phi_l0), float(phi_r0)])
    err = np.linalg.norm(q_m - q_m0)

    iter_count = 0
    while err > tol and iter_count < max_iter:
        J = numba_util.compute_jacobian(q_j[0], q_j[1], d1, d2, h1, h2, r1, r2, u_x, u_z)  # Compute Jacobian matrix
        J_inv = numba_util.fast_2x2_inverse(J)  # Fast inverse

        delta_q_m = q_m - q_m0  # Motor angle error

        q_j = q_j + alpha * (J_inv @ delta_q_m) # Update joint variables
        phi_l_new, phi_r_new = numba_util.double_link_inverse(q_j[0], q_j[1], d1, d2, h1, h2, r1, r2, u_x, u_z)
        q_m0 = np.array([phi_l_new, phi_r_new])
        err = np.linalg.norm(delta_q_m) # Update error

        iter_count += 1
    return q_j, err, iter_count


class DoubleLinkSolver(BaseSolver):
    def __init__(self, solver_params: dict):
        super().__init__(solver_params)
        self._last_joint_pos = np.zeros(2)

    def joint2motor_pos(self, joint_pos: np.ndarray):
        """
        Position mapping: motor_pos = f(joint_pos)
        where f is the inverse function of the joint2motor_pos function

        Args:
            joint_pos: Current joint positions
        """
        assert len(joint_pos) == 2, f"Joint position must have 2 elements: {len(joint_pos)}"
        pitch, roll = joint_pos
        phi_l, phi_r = numba_util.double_link_inverse(pitch, roll, **self.solver_params)
        motor_pos = np.array([float(phi_l), float(phi_r)])
        return motor_pos

    def motor2joint_pos(self, motor_pos: np.ndarray):
        """
        Forward kinematics: compute joint positions from motor positions
        Using numerical optimization

        Args:
            motor_pos: Motor positions
        """
        q_j, err, iter_count = solve_ik_numba(self._last_joint_pos, motor_pos, **self.solver_params)
        self._last_joint_pos = q_j
        return q_j

    def joint2motor_vel(self, joint_pos: np.ndarray, joint_vel: np.ndarray):
        """
        Velocity mapping: motor_vel = J * joint_vel
        where J is the Jacobian matrix d(motor_pos)/d(joint_pos)
        
        Args:
            joint_pos: Current joint positions (used to compute Jacobian)
            joint_vel: Joint velocities
        """
        pitch, roll = joint_pos
        J = np.array(numba_util.compute_jacobian(pitch, roll, **self.solver_params))
        motor_vel = J @ joint_vel
        return motor_vel

    def motor2joint_vel(self, joint_pos: np.ndarray, motor_vel: np.ndarray):
        """
        Inverse velocity mapping: joint_vel = J^{-1} * motor_vel

        Args:
            joint_pos: Current joint positions (used to compute Jacobian)
            motor_vel: Motor velocities
        """
        pitch, roll = joint_pos
        J = np.array(numba_util.compute_jacobian(pitch, roll, **self.solver_params))
        joint_vel = numba_util.fast_2x2_inverse(J) @ motor_vel
        return joint_vel

    def joint2motor_torque(self, joint_pos: np.ndarray, joint_torque: np.ndarray):
        """
        Torque mapping: motor_torque = J^{-T} * joint_torque

        Args:
            joint_pos: Current joint positions (used to compute Jacobian)
            joint_torque: Joint torques
        """
        pitch, roll = joint_pos
        J = np.array(numba_util.compute_jacobian(pitch, roll, **self.solver_params))
        motor_torque = numba_util.fast_2x2_inverse(J.T) @ joint_torque
        return motor_torque

    def motor2joint_torque(self, joint_pos: np.ndarray, motor_torque: np.ndarray):
        """
        Inverse torque mapping: joint_torque = J^T * motor_torque
        
        Args:
            joint_pos: Current joint positions (used to compute Jacobian)
            motor_torque: Motor torques
        """
        pitch, roll = joint_pos
        J = np.array(numba_util.compute_jacobian(pitch, roll, **self.solver_params))
        joint_torque = J.T @ motor_torque
        return joint_torque

    def joint2motor(self, joint_pos, joint_vel, joint_torque):
        motor_pos = self.joint2motor_pos(joint_pos)
        pitch, roll = joint_pos
        J = np.array(numba_util.compute_jacobian(pitch, roll, **self.solver_params))
        motor_vel = J @ joint_vel
        motor_torque = np.linalg.solve(J.T, joint_torque)
        return motor_pos, motor_vel, motor_torque

    def motor2joint(self, motor_pos, motor_vel, motor_torque):
        joint_pos = self.motor2joint_pos(motor_pos)
        pitch, roll = joint_pos
        J = np.array(numba_util.compute_jacobian(pitch, roll, **self.solver_params))
        joint_vel = np.linalg.solve(J, motor_vel)
        joint_torque = J.T @ motor_torque
        return joint_pos, joint_vel, joint_torque
