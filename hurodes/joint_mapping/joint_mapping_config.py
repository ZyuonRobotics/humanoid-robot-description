import numpy as np
from typing import Any

from pydantic import Field
from hurodes.utils.config import BaseConfig
from hurodes.joint_mapping import solver_dict


class SolverConfig(BaseConfig):
    solver_type: str = ""
    solver_params: dict = {}
    joint_idx_list: list[int] = []
    motor_idx_list: list[int] = []
    solver: Any = Field(default=None, init=False, exclude=True)

    def model_post_init(self, __context: Any) -> None:
        assert self.solver_type in solver_dict, f"Invalid solver type: {self.solver_type}"
        self.solver = solver_dict[self.solver_type](self.solver_params)
        assert len(self.joint_idx_list) == len(self.motor_idx_list), f"Number of joint indices must match number of motor indices: {len(self.joint_idx_list)} != {len(self.motor_idx_list)}"

class JointMappingConfig(BaseConfig):
    """
    # The mapping consists of three parts:
    # 1. Mapping between motor IDs and motor indices.
    # 2. Mapping of the direction for motor indices.
    # 3. Mapping between motor indices and joint indices (currently only parallel linkage mapping is supported).
    # For reading: First apply the direction mapping, then use the parallel linkage mapping to get the joint index (if parallel linkage mapping exists).
    # For writing: First use the parallel linkage mapping to get the motor index (if parallel linkage mapping exists), then apply the direction mapping.
    """
    model_config = {"arbitrary_types_allowed": True}
    motor_id_list: list[int] = []
    solver_config_dict: dict[str, SolverConfig] = {}

    negative_motor_idx_list: list[int] = []
    oritation_mapping: np.ndarray = Field(default=None, init=False, exclude=True)

    @property
    def motor_num(self):
        return len(self.motor_id_list)

    def model_post_init(self, __context: any):
        assert len(self.motor_id_list) == len(set(self.motor_id_list)), f"Motor IDs must be unique: {self.motor_id_list}"
        assert len(self.negative_motor_idx_list) == len(set(self.negative_motor_idx_list)), f"Negative list must be unique: {self.negative_motor_idx_list}"
        # generate_oritation_mapping
        self.oritation_mapping = np.where(np.isin(np.arange(len(self.motor_id_list)), self.negative_motor_idx_list), -1, 1)

        motor_found = np.zeros(self.motor_num, dtype=bool)
        joint_found = np.zeros(self.motor_num, dtype=bool)
        for solver_config in self.solver_config_dict.values():
            assert not any(motor_found[solver_config.motor_idx_list]), f"Motor index {solver_config.motor_idx_list} is already assigned to another solver"
            assert not any(joint_found[solver_config.joint_idx_list]), f"Joint index {solver_config.joint_idx_list} is already assigned to another solver"
            motor_found[solver_config.motor_idx_list] = True
            joint_found[solver_config.joint_idx_list] = True
        
        assert all(motor_found == joint_found), f"Motor and joint indices must match: {motor_found} != {joint_found}"

    def joint2motor_pos(self, joint_pos: np.ndarray):
        res = joint_pos.copy()
        for solver_config in self.solver_config_dict.values():
            res[solver_config.motor_idx_list] = solver_config.solver.joint2motor_pos(joint_pos[solver_config.joint_idx_list])
        res = self.oritation_transform(res)
        return res

    def motor2joint_pos(self, motor_pos: np.ndarray):
        motor_pos = self.oritation_transform(motor_pos)
        res = motor_pos.copy()
        for solver_config in self.solver_config_dict.values():
            res[solver_config.joint_idx_list] = solver_config.solver.motor2joint_pos(motor_pos[solver_config.motor_idx_list])
        return res

    def joint2motor_vel(self, joint_pos: np.ndarray, joint_vel: np.ndarray):
        res = joint_vel.copy()
        for solver_config in self.solver_config_dict.values():
            res[solver_config.motor_idx_list] = solver_config.solver.joint2motor_vel(joint_pos[solver_config.joint_idx_list], joint_vel[solver_config.joint_idx_list])
        res = self.oritation_transform(res)
        return res

    def motor2joint_vel(self, joint_pos: np.ndarray, motor_vel: np.ndarray):
        motor_vel = self.oritation_transform(motor_vel)
        res = motor_vel.copy()
        for solver_config in self.solver_config_dict.values():
            res[solver_config.joint_idx_list] = solver_config.solver.motor2joint_vel(joint_pos[solver_config.joint_idx_list], motor_vel[solver_config.motor_idx_list])
        return res

    def joint2motor_torque(self, joint_pos: np.ndarray, joint_torque: np.ndarray):
        res = joint_torque.copy()
        for solver_config in self.solver_config_dict.values():
            res[solver_config.motor_idx_list] = solver_config.solver.joint2motor_torque(joint_pos[solver_config.joint_idx_list], joint_torque[solver_config.joint_idx_list])
        res = self.oritation_transform(res)
        return res

    def motor2joint_torque(self, joint_pos: np.ndarray, motor_torque: np.ndarray):
        motor_torque = self.oritation_transform(motor_torque)
        res = motor_torque.copy()
        for solver_config in self.solver_config_dict.values():
            res[solver_config.joint_idx_list] = solver_config.solver.motor2joint_torque(joint_pos[solver_config.joint_idx_list], motor_torque[solver_config.motor_idx_list])
        return res

    def motor2joint(self, motor_pos: np.ndarray, motor_vel: np.ndarray, motor_torque: np.ndarray):
        joint_pos = self.motor2joint_pos(motor_pos)
        joint_vel = self.motor2joint_vel(joint_pos, motor_vel)
        joint_torque = self.motor2joint_torque(joint_pos, motor_torque)
        return joint_pos, joint_vel, joint_torque

    def joint2motor(self, joint_pos: np.ndarray, joint_vel: np.ndarray, joint_torque: np.ndarray):
        motor_pos = self.joint2motor_pos(joint_pos)
        motor_vel = self.joint2motor_vel(joint_pos, joint_vel)
        motor_torque = self.joint2motor_torque(joint_pos, joint_torque)
        return motor_pos, motor_vel, motor_torque

    def oritation_transform(self, value_array: np.ndarray):
        # apply the oritation mapping to the value array(position, velocity or torque) in the motor space or the joint space
        assert len(value_array) == len(self.motor_id_list), f"Value array must have {len(self.motor_id_list)}, got {len(value_array)}."
        return self.oritation_mapping * value_array

if __name__ == "__main__":
    from hurodes import ROBOTS_PATH
    from time import perf_counter
    config = JointMappingConfig.from_yaml(ROBOTS_PATH / "zhaplin-21dof" / "joint_mapping.yaml")
    print(f"config: {config}")
    
    joint_pos = np.pi * np.random.rand(config.motor_num)
    # limit the joint positions
    joint_pos[4] = np.clip(joint_pos[4], -0.9, 0.9)
    joint_pos[5] = np.clip(joint_pos[5], -0.9, 0.9)
    joint_pos[10] = np.clip(joint_pos[10], -0.9, 0.9)
    joint_pos[11] = np.clip(joint_pos[11], -0.9, 0.9)
    print(f"joint_pos[4:6]: {joint_pos[4:6]}")
    print(f"joint_pos[10:12]: {joint_pos[10:12]}")
    joint_vel = np.random.rand(config.motor_num)
    joint_torque = np.random.rand(config.motor_num)

    print("=" * 60)
    print("single test")

    N = 1000
    time0 = perf_counter()
    for _ in range(N):
        motor_pos = config.joint2motor_pos(joint_pos)
        motor_vel = config.joint2motor_vel(joint_pos, joint_vel)
        motor_torque = config.joint2motor_torque(joint_pos, joint_torque)
    time1 = perf_counter()
    time_cost = time1 - time0
    print(f"Time cost joint2motor: {1e3*time_cost/N:.6f} ms")

    time0 = perf_counter()
    for _ in range(N):
        recovered_joint_pos = config.motor2joint_pos(motor_pos)
        recovered_joint_vel = config.motor2joint_vel(joint_pos, motor_vel)
        recovered_joint_torque = config.motor2joint_torque(joint_pos, motor_torque)
    time1 = perf_counter()
    time_cost = time1 - time0
    print(f"Time cost motor2joint: {1e3*time_cost/N:.6f} ms")


    print(f"joint_pos: {joint_pos[3:5]}")
    print(f"motor_pos: {motor_pos[3:5]}")
    print(f"recovered: {recovered_joint_pos[3:5]}")

    print(np.linalg.norm(joint_pos - recovered_joint_pos))
    print(np.linalg.norm(joint_vel - recovered_joint_vel))
    print(np.linalg.norm(joint_torque - recovered_joint_torque))

    print("=" * 60)
    print("batch test")

    time0 = perf_counter()
    for _ in range(N):
        recovered_motor_pos, recovered_motor_vel, recovered_motor_torque = config.joint2motor(joint_pos, joint_vel, joint_torque)
    time1 = perf_counter()
    time_cost = time1 - time0
    print(f"Time cost motor2joint: {1e3*time_cost/N:.6f} ms")

    time0 = perf_counter()
    for _ in range(N):
        recovered_joint_pos, recovered_joint_vel, recovered_joint_torque = config.motor2joint(recovered_motor_pos, recovered_motor_vel, recovered_motor_torque)
    time1 = perf_counter()
    time_cost = time1 - time0
    print(f"Time cost joint2motor: {1e3*time_cost/N:.6f} ms")

    print(f"joint_pos: {joint_pos[:5]}")
    print(f"motor_pos: {recovered_motor_pos[:5]}")
    print(f"recovered: {recovered_joint_pos[:5]}")

    print(np.linalg.norm(joint_pos - recovered_joint_pos))
    print(np.linalg.norm(joint_vel - recovered_joint_vel))
    print(np.linalg.norm(joint_torque - recovered_joint_torque))
