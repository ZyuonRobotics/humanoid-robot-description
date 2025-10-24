from turtle import pd
import numpy as np
from typing import Any

from pydantic import Field
from hurodes.utils.config import BaseConfig
from hurodes.hrdf.joint_mapping import solver_dict


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
    motor_id_list: list[int] = []
    solver_config_dict: dict[str, SolverConfig] = {}

    @property
    def motor_num(self):
        return len(self.motor_id_list)

    def model_post_init(self, __context: any):
        motor_found = np.zeros(self.motor_num, dtype=bool)
        joint_found = np.zeros(self.motor_num, dtype=bool)
        for solver_config in self.solver_config_dict.values():
            assert not any(motor_found[solver_config.motor_idx_list]), f"Motor index {solver_config.motor_idx_list} is already assigned to another solver"
            assert not any(joint_found[solver_config.joint_idx_list]), f"Joint index {solver_config.joint_idx_list} is already assigned to another solver"
            motor_found[solver_config.motor_idx_list] = True
            joint_found[solver_config.joint_idx_list] = True
        
        assert all(motor_found == joint_found), f"Motor and joint indices must match: {motor_found} != {joint_found}"
        if (~motor_found).sum() > 0:
            self.solver_config_dict["direct"] = SolverConfig(
                solver_type="direct", 
                solver_params={}, 
                joint_idx_list=np.where(~motor_found)[0].tolist(), 
                motor_idx_list=np.where(~joint_found)[0].tolist()
            )

    def joint2motor_pos(self, joint_pos: np.ndarray):
        res = np.zeros_like(joint_pos)
        for solver_config in self.solver_config_dict.values():
            res[solver_config.joint_idx_list] = solver_config.solver.joint2motor_pos(joint_pos[solver_config.joint_idx_list])
        return res

    def motor2joint_pos(self, motor_pos: np.ndarray):
        res = np.zeros_like(motor_pos)
        for solver_config in self.solver_config_dict.values():
            res[solver_config.motor_idx_list] = solver_config.solver.motor2joint_pos(motor_pos[solver_config.motor_idx_list])
        return res

    def joint2motor_vel(self, joint_pos: np.ndarray, joint_vel: np.ndarray):
        res = np.zeros_like(joint_vel)
        for solver_config in self.solver_config_dict.values():
            res[solver_config.joint_idx_list] = solver_config.solver.joint2motor_vel(joint_pos[solver_config.joint_idx_list], joint_vel[solver_config.joint_idx_list])
        return res

    def motor2joint_vel(self, joint_pos: np.ndarray, motor_vel: np.ndarray):
        res = np.zeros_like(motor_vel)
        for solver_config in self.solver_config_dict.values():
            res[solver_config.motor_idx_list] = solver_config.solver.motor2joint_vel(joint_pos[solver_config.joint_idx_list], motor_vel[solver_config.motor_idx_list])
        return res

    def joint2motor_torque(self, joint_pos: np.ndarray, joint_torque: np.ndarray):
        res = np.zeros_like(joint_torque)
        for solver_config in self.solver_config_dict.values():
            res[solver_config.joint_idx_list] = solver_config.solver.joint2motor_torque(joint_pos[solver_config.joint_idx_list], joint_torque[solver_config.joint_idx_list])
        return res

    def motor2joint_torque(self, joint_pos: np.ndarray, motor_torque: np.ndarray):
        res = np.zeros_like(motor_torque)
        for solver_config in self.solver_config_dict.values():
            res[solver_config.motor_idx_list] = solver_config.solver.motor2joint_torque(joint_pos[solver_config.joint_idx_list], motor_torque[solver_config.motor_idx_list])
        return res

if __name__ == "__main__":
    from hurodes import ROBOTS_PATH
    config = JointMappingConfig.from_yaml(ROBOTS_PATH / "zhaplin-19dof" / "joint_mapping.yaml")
    
    joint_pos = np.random.rand(config.motor_num)
    joint_vel = np.random.rand(config.motor_num)
    joint_torque = np.random.rand(config.motor_num)

    motor_pos = config.joint2motor_pos(joint_pos)
    motor_vel = config.joint2motor_vel(joint_pos, joint_vel)
    motor_torque = config.joint2motor_torque(joint_pos, joint_torque)

    recovered_joint_pos = config.motor2joint_pos(motor_pos)
    recovered_joint_vel = config.motor2joint_vel(joint_pos, motor_vel)
    recovered_joint_torque = config.motor2joint_torque(joint_pos, motor_torque)

    print(np.linalg.norm(joint_pos - recovered_joint_pos))
    print(np.linalg.norm(joint_vel - recovered_joint_vel))
    print(np.linalg.norm(joint_torque - recovered_joint_torque))
