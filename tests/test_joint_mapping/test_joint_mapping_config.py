import numpy as np
from time import perf_counter
from hurodes.joint_mapping.joint_mapping_config import JointMappingConfig
from hurodes import ROBOTS_PATH


def test_joint_mapping_config_integration():
    """Integration test for JointMappingConfig with real robot configuration"""
    config = JointMappingConfig.from_yaml(ROBOTS_PATH / "zhaplin-21dof" / "joint_mapping.yaml")

    # Generate test joint positions
    joint_pos = np.pi * np.random.rand(config.motor_num)

    # Apply joint limits for specific indices (as in original test)
    joint_pos[4] = np.clip(joint_pos[4], -0.9, 0.9)
    joint_pos[5] = np.clip(joint_pos[5], -0.9, 0.9)
    joint_pos[10] = np.clip(joint_pos[10], -0.9, 0.9)
    joint_pos[11] = np.clip(joint_pos[11], -0.9, 0.9)

    joint_vel = np.random.rand(config.motor_num)
    joint_torque = np.random.rand(config.motor_num)

    # Test single conversions
    motor_pos = config.joint2motor_pos(joint_pos)
    motor_vel = config.joint2motor_vel(joint_pos, joint_vel)
    motor_torque = config.joint2motor_torque(joint_pos, joint_torque)

    recovered_joint_pos = config.motor2joint_pos(motor_pos)
    recovered_joint_vel = config.motor2joint_vel(joint_pos, motor_vel)
    recovered_joint_torque = config.motor2joint_torque(joint_pos, motor_torque)

    # Verify conversions are accurate (within reasonable tolerance)
    pos_error = np.linalg.norm(joint_pos - recovered_joint_pos)
    vel_error = np.linalg.norm(joint_vel - recovered_joint_vel)
    torque_error = np.linalg.norm(joint_torque - recovered_joint_torque)

    # Position mapping may have some numerical error due to solver approximations
    assert pos_error < 1e-2, f"Position mapping error too large: {pos_error}"
    assert vel_error < 1e-6, f"Velocity mapping error too large: {vel_error}"
    assert torque_error < 1e-6, f"Torque mapping error too large: {torque_error}"

    # Test batch conversions
    recovered_motor_pos, recovered_motor_vel, recovered_motor_torque = config.joint2motor(joint_pos, joint_vel, joint_torque)
    recovered_joint_pos_batch, recovered_joint_vel_batch, recovered_joint_torque_batch = config.motor2joint(
        recovered_motor_pos, recovered_motor_vel, recovered_motor_torque
    )

    # Verify batch conversions match single conversions
    assert np.allclose(motor_pos, recovered_motor_pos, atol=1e-6), "Batch motor_pos doesn't match single conversion"
    assert np.allclose(motor_vel, recovered_motor_vel, atol=1e-6), "Batch motor_vel doesn't match single conversion"
    assert np.allclose(motor_torque, recovered_motor_torque, atol=1e-6), "Batch motor_torque doesn't match single conversion"

    assert np.allclose(recovered_joint_pos, recovered_joint_pos_batch, atol=1e-6), "Batch joint_pos recovery doesn't match single conversion"
    assert np.allclose(recovered_joint_vel, recovered_joint_vel_batch, atol=1e-6), "Batch joint_vel recovery doesn't match single conversion"
    assert np.allclose(recovered_joint_torque, recovered_joint_torque_batch, atol=1e-6), "Batch joint_torque recovery doesn't match single conversion"


def test_joint_mapping_performance():
    """Performance test for JointMappingConfig conversions"""
    config = JointMappingConfig.from_yaml(ROBOTS_PATH / "zhaplin-21dof" / "joint_mapping.yaml")

    joint_pos = np.pi * np.random.rand(config.motor_num)
    joint_pos[4] = np.clip(joint_pos[4], -0.9, 0.9)
    joint_pos[5] = np.clip(joint_pos[5], -0.9, 0.9)
    joint_pos[10] = np.clip(joint_pos[10], -0.9, 0.9)
    joint_pos[11] = np.clip(joint_pos[11], -0.9, 0.9)

    joint_vel = np.random.rand(config.motor_num)
    joint_torque = np.random.rand(config.motor_num)

    N = 100

    # Test joint2motor performance
    start_time = perf_counter()
    for _ in range(N):
        motor_pos = config.joint2motor_pos(joint_pos)
        motor_vel = config.joint2motor_vel(joint_pos, joint_vel)
        motor_torque = config.joint2motor_torque(joint_pos, joint_torque)
    joint2motor_time = perf_counter() - start_time

    # Test motor2joint performance
    start_time = perf_counter()
    for _ in range(N):
        recovered_joint_pos = config.motor2joint_pos(motor_pos)
        recovered_joint_vel = config.motor2joint_vel(joint_pos, motor_vel)
        recovered_joint_torque = config.motor2joint_torque(joint_pos, motor_torque)
    motor2joint_time = perf_counter() - start_time

    # Test batch performance
    start_time = perf_counter()
    for _ in range(N):
        config.joint2motor(joint_pos, joint_vel, joint_torque)
    batch_joint2motor_time = perf_counter() - start_time

    start_time = perf_counter()
    for _ in range(N):
        config.motor2joint(motor_pos, motor_vel, motor_torque)
    batch_motor2joint_time = perf_counter() - start_time

    # Performance should be reasonable (less than 1ms per conversion for this robot)
    avg_joint2motor_time = joint2motor_time / N * 1000  # ms
    avg_motor2joint_time = motor2joint_time / N * 1000  # ms
    avg_batch_joint2motor_time = batch_joint2motor_time / N * 1000  # ms
    avg_batch_motor2joint_time = batch_motor2joint_time / N * 1000  # ms

    # These are performance assertions - adjust thresholds based on requirements
    assert avg_joint2motor_time < 10.0, f"joint2motor too slow: {avg_joint2motor_time:.3f} ms"
    assert avg_motor2joint_time < 10.0, f"motor2joint too slow: {avg_motor2joint_time:.3f} ms"
    assert avg_batch_joint2motor_time < 10.0, f"batch joint2motor too slow: {avg_batch_joint2motor_time:.3f} ms"
    assert avg_batch_motor2joint_time < 10.0, f"batch motor2joint too slow: {avg_batch_motor2joint_time:.3f} ms"
