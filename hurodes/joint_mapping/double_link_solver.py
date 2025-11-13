import numpy as np
import time
try:
    from numba import jit
    NUMBA_AVAILABLE = True
except ImportError:
    NUMBA_AVAILABLE = False
    jit = lambda x: x  # fallback to no-op decorator

try:
    import casadi as ca
    CASADI_AVAILABLE = True
except ImportError:
    CASADI_AVAILABLE = False
    ca = None

from hurodes.joint_mapping.base_solver import BaseSolver
from hurodes.joint_mapping import numba_util, casadi_uitl


def solve_ik_numba(initial_pos, q_m, jac_func, solver_params, max_iter=100, tol=1e-4, alpha=0.9):
    """
    Solve inverse problem using Jacobian matrix iteration
    Parameters:
        q_j0: Initial joint variables (array shaped like [q1, q2])
        q_m: Target motor angles (array shaped like [phi_l, phi_r])
        solver_params: Solver parameters, including d1, d2, h1, h2, r1, r2, u_x, u_z
        jac_func: Function to compute Jacobian matrix (Numba compatible), takes (q1, q2) as input, returns 2x2 array
        max_iter: Maximum number of iterations
        tol: Error tolerance
        alpha: Step size parameter (damping factor to avoid oscillation)

    Returns:
        q_j: Optimized joint variables
        err: Final error
        iter_count: Actual number of iterations
    """
    q_j = initial_pos.copy()  # Avoid modifying original data
    phi_l0, phi_r0 = numba_util.double_link_inverse(float(q_j[0]), float(q_j[1]), **solver_params)
    q_m0 = np.array([float(phi_l0), float(phi_r0)])
    err = np.linalg.norm(q_m - q_m0)

    iter_count = 0
    while err > tol and iter_count < max_iter:
        J = np.array(jac_func(q_j[0], q_j[1]))  # Compute Jacobian matrix
        q_j, q_m0, err = func(J, q_m, q_m0, q_j, alpha, **solver_params)
        iter_count += 1
    return q_j, err, iter_count

@jit(nopython=True, fastmath=True, cache=True)
def func(J, q_m, q_m0, q_j, alpha, d1, d2, h1, h2, r1, r2, u_x, u_z):
    J_inv = numba_util.fast_2x2_inverse(J)  # Fast inverse
    delta_q_m = q_m - q_m0  # Motor angle error
    q_j = q_j + alpha * (J_inv @ delta_q_m) # Update joint variables
    phi_l_new, phi_r_new = numba_util.double_link_inverse(q_j[0], q_j[1], d1, d2, h1, h2, r1, r2, u_x, u_z)
    q_m0 = np.array([phi_l_new, phi_r_new])
    err = np.linalg.norm(delta_q_m) # Update error
    return q_j, q_m0, err


class DoubleLinkSolver(BaseSolver):
    def __init__(self, solver_params: dict):
        if not CASADI_AVAILABLE:
            raise ImportError(
                "CasADi is required for DoubleLinkSolver. "
                "Please install it with: pip install 'hurodes[hal]' or pip install casadi"
            )
        super().__init__(solver_params)
        self.jacobian_func = self._generate_jacobian_func()
        self.last_joint_pos = np.zeros(2)

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

    def _generate_jacobian_func(self):
        roll = ca.SX.sym('roll')
        pitch = ca.SX.sym('pitch')
        phi_l, phi_r = casadi_uitl.double_link_inverse(pitch, roll, **self.solver_params)
        jac = ca.jacobian(ca.vertcat(phi_l, phi_r), ca.vertcat(pitch, roll))
        func = ca.Function('jacobian', [pitch, roll], [jac])
        return func

    def motor2joint_pos(self, motor_pos: np.ndarray):
        """
        Forward kinematics: compute joint positions from motor positions
        Using numerical optimization

        Args:
            motor_pos: Motor positions
        """
        q_j, err, iter_count = solve_ik_numba(self.last_joint_pos, motor_pos, self.jacobian_func, self.solver_params)
        self.last_joint_pos = q_j
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
        J = np.array(self.jacobian_func(pitch, roll))
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
        J = np.array(self.jacobian_func(pitch, roll))
        joint_vel = np.linalg.solve(J, motor_vel)
        return joint_vel

    def joint2motor_torque(self, joint_pos: np.ndarray, joint_torque: np.ndarray):
        """
        Torque mapping: motor_torque = J^{-T} * joint_torque
        
        Args:
            joint_pos: Current joint positions (used to compute Jacobian)
            joint_torque: Joint torques
        """
        pitch, roll = joint_pos
        J = np.array(self.jacobian_func(pitch, roll))
        motor_torque = np.linalg.solve(J.T, joint_torque)
        return motor_torque

    def motor2joint_torque(self, joint_pos: np.ndarray, motor_torque: np.ndarray):
        """
        Inverse torque mapping: joint_torque = J^T * motor_torque
        
        Args:
            joint_pos: Current joint positions (used to compute Jacobian)
            motor_torque: Motor torques
        """
        pitch, roll = joint_pos
        J = np.array(self.jacobian_func(pitch, roll))
        joint_torque = J.T @ motor_torque
        return joint_torque


if __name__ == "__main__":
    solver_params = {
        "d1": 0.035 / 2.0,
        "d2": 0.035 / 2.0,
        "h1": 0.10,
        "h2": 0.17,
        "r1": 0.04,
        "r2": 0.04,
        "u_x": -0.0445,
        "u_z": 0.00,
    }
    
    solver = DoubleLinkSolver(solver_params)

    # Test position mapping
    print("=" * 60)
    print("Test Position Mapping")
    joint_pos = np.array([0.1, 0.05])
    print(f"Joint position (pitch, roll): {joint_pos}")
    motor_pos = solver.joint2motor_pos(joint_pos)
    print(f"Motor position (phi_l, phi_r): {motor_pos}")
    
    # Test forward kinematics
    recovered_joint_pos = solver.motor2joint_pos(motor_pos)
    print(f"Recovered joint position: {recovered_joint_pos}")
    print(f"Error: {np.linalg.norm(joint_pos - recovered_joint_pos)}")
    
    # Test velocity mapping
    print("\n" + "=" * 60)
    print("Test Velocity Mapping")
    joint_vel = np.array([1.0, 0.5])
    print(f"Joint velocity: {joint_vel}")
    motor_vel = solver.joint2motor_vel(joint_pos, joint_vel)
    print(f"Motor velocity: {motor_vel}")
    recovered_joint_vel = solver.motor2joint_vel(joint_pos, motor_vel)
    print(f"Recovered joint velocity: {recovered_joint_vel}")
    print(f"Error: {np.linalg.norm(joint_vel - recovered_joint_vel)}")
    
    # Test torque mapping
    print("\n" + "=" * 60)
    print("Test Torque Mapping")
    joint_torque = np.array([2.0, 1.0])
    print(f"Joint torque: {joint_torque}")
    motor_torque = solver.joint2motor_torque(joint_pos, joint_torque)
    print(f"Motor torque: {motor_torque}")
    recovered_joint_torque = solver.motor2joint_torque(joint_pos, motor_torque)
    print(f"Recovered joint torque: {recovered_joint_torque}")
    print(f"Error: {np.linalg.norm(joint_torque - recovered_joint_torque)}")
    
    # Verify virtual work principle
    print("\n" + "=" * 60)
    print("Verify Virtual Work Principle (should be equal)")
    print(f"Joint virtual work: {np.dot(joint_torque, joint_vel)}")
    print(f"Motor virtual work: {np.dot(motor_torque, motor_vel)}")
    print(f"Virtual work difference: {abs(np.dot(joint_torque, joint_vel) - np.dot(motor_torque, motor_vel))}")
