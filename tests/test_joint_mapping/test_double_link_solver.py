import pytest
import numpy as np
from hurodes.joint_mapping.double_link_solver import DoubleLinkSolver


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


@pytest.fixture
def solver(solver_params):
    """Fixture providing DoubleLinkSolver instance"""
    return DoubleLinkSolver(solver_params)


def test_position_mapping(solver):
    """Test position mapping: joint_pos <-> motor_pos"""
    joint_pos = np.array([0.1, 0.05])
    motor_pos = solver.joint2motor_pos(joint_pos)
    recovered_joint_pos = solver.motor2joint_pos(motor_pos)
    
    # Verify recovered joint position is close to original
    error = np.linalg.norm(joint_pos - recovered_joint_pos)
    assert error < 1e-3, f"Position mapping error too large: {error}"


def test_velocity_mapping(solver):
    """Test velocity mapping: joint_vel <-> motor_vel"""
    joint_pos = np.array([0.1, 0.05])
    joint_vel = np.array([1.0, 0.5])
    
    motor_vel = solver.joint2motor_vel(joint_pos, joint_vel)
    recovered_joint_vel = solver.motor2joint_vel(joint_pos, motor_vel)
    
    # Verify recovered joint velocity is close to original
    error = np.linalg.norm(joint_vel - recovered_joint_vel)
    assert error < 1e-6, f"Velocity mapping error too large: {error}"


def test_torque_mapping(solver):
    """Test torque mapping: joint_torque <-> motor_torque"""
    joint_pos = np.array([0.1, 0.05])
    joint_torque = np.array([2.0, 1.0])
    
    motor_torque = solver.joint2motor_torque(joint_pos, joint_torque)
    recovered_joint_torque = solver.motor2joint_torque(joint_pos, motor_torque)
    
    # Verify recovered joint torque is close to original
    error = np.linalg.norm(joint_torque - recovered_joint_torque)
    assert error < 1e-6, f"Torque mapping error too large: {error}"


def test_virtual_work_principle(solver):
    """Verify virtual work principle: joint_torque^T * joint_vel = motor_torque^T * motor_vel"""
    joint_pos = np.array([0.1, 0.05])
    joint_vel = np.array([1.0, 0.5])
    joint_torque = np.array([2.0, 1.0])
    
    motor_vel = solver.joint2motor_vel(joint_pos, joint_vel)
    motor_torque = solver.joint2motor_torque(joint_pos, joint_torque)
    
    joint_virtual_work = np.dot(joint_torque, joint_vel)
    motor_virtual_work = np.dot(motor_torque, motor_vel)
    
    # Virtual work should be equal (within numerical precision)
    difference = abs(joint_virtual_work - motor_virtual_work)
    assert difference < 1e-6, f"Virtual work principle violated: difference = {difference}"


def test_joint2motor_composite(solver):
    """Test composite joint2motor mapping"""
    joint_pos = np.array([0.1, 0.05])
    joint_vel = np.array([1.0, 0.5])
    joint_torque = np.array([2.0, 1.0])
    
    motor_pos, motor_vel, motor_torque = solver.joint2motor(joint_pos, joint_vel, joint_torque)
    recovered_joint_pos, recovered_joint_vel, recovered_joint_torque = solver.motor2joint(motor_pos, motor_vel, motor_torque)

    error = np.linalg.norm(joint_pos - recovered_joint_pos)
    assert error < 1e-3, f"Position recovery error too large: {error}"
    error = np.linalg.norm(joint_vel - recovered_joint_vel)
    assert error < 1e-3, f"Velocity recovery error too large: {error}"
    error = np.linalg.norm(joint_torque - recovered_joint_torque)
    assert error < 1e-3, f"Torque recovery error too large: {error}"
