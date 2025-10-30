import numpy as np
from scipy.optimize import least_squares

try:
    import casadi as ca
    CASADI_AVAILABLE = True
except ImportError:
    CASADI_AVAILABLE = False
    ca = None

from hurodes.hrdf.joint_mapping.base_solver import BaseSolver

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

class DoubleLinkSolver(BaseSolver):
    def __init__(self, solver_params: dict):
        if not CASADI_AVAILABLE:
            raise ImportError(
                "CasADi is required for DoubleLinkSolver. "
                "Please install it with: pip install 'hurodes[hal]' or pip install casadi"
            )
        super().__init__(solver_params)
        self.jacobian_func = self._generate_jacobian_func()

    def inverse(self, pitch, roll):
        p_lu_3 = ca.vertcat(self.solver_params["u_x"], +self.solver_params["d1"], self.solver_params["u_z"])
        p_ru_3 = ca.vertcat(self.solver_params["u_x"], -self.solver_params["d2"], self.solver_params["u_z"])
        p_la_1 = ca.vertcat(0, +self.solver_params["d1"], self.solver_params["h1"])
        p_ra_1 = ca.vertcat(0, -self.solver_params["d2"], self.solver_params["h2"])
        
        p_lu_1 = euler_to_rotmat(roll, pitch, 0) @ p_lu_3
        p_ru_1 = euler_to_rotmat(roll, pitch, 0) @ p_ru_3
        phi_l = get_phi(p_lu_1, p_la_1, self.solver_params["h1"], self.solver_params["r1"])
        phi_r = get_phi(p_ru_1, p_ra_1, self.solver_params["h2"], self.solver_params["r2"])
        return phi_l, phi_r

    def joint2motor_pos(self, joint_pos: np.ndarray):
        """
        Position mapping: motor_pos = f(joint_pos)
        where f is the inverse function of the joint2motor_pos function
        
        Args:
            joint_pos: Current joint positions
        """
        assert len(joint_pos) == 2, f"Joint position must have 2 elements: {len(joint_pos)}"
        pitch, roll = joint_pos
        phi_l, phi_r = self.inverse(pitch, roll)
        motor_pos = np.array([float(phi_l), float(phi_r)])
        return motor_pos

    def _generate_jacobian_func(self):
        roll = ca.SX.sym('roll')
        pitch = ca.SX.sym('pitch')
        phi_l, phi_r = self.inverse(pitch, roll)
        jac = ca.jacobian(ca.vertcat(phi_l, phi_r), ca.vertcat(pitch, roll))
        func = ca.Function('jacobian', [pitch, roll], [jac])
        return func

    def motor2joint_pos(self, motor_pos: np.ndarray, initial_guess: np.ndarray = None):
        """
        Forward kinematics: compute joint positions from motor positions
        Using numerical optimization
        
        Args:
            motor_pos: Motor positions
            initial_guess: Initial guess for optimization, defaults to zero vector if not provided
        """        
        if initial_guess is None:
            initial_guess = np.zeros(2)
        
        def residual(joint_pos):
            computed_motor_pos = self.joint2motor_pos(joint_pos)
            return computed_motor_pos - motor_pos
        
        result = least_squares(residual, initial_guess, method='lm')
        joint_pos = result.x
        return joint_pos

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
