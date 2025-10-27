from abc import ABC, abstractmethod

import numpy as np



class BaseSolver(ABC):
    def __init__(self, solver_params: dict):
        self.solver_params = solver_params

    @abstractmethod
    def joint2motor_pos(self, joint_pos: np.ndarray):
        raise NotImplementedError

    @abstractmethod
    def motor2joint_pos(self, motor_pos: np.ndarray):
        raise NotImplementedError

    @abstractmethod
    def joint2motor_vel(self, joint_pos: np.ndarray, joint_vel: np.ndarray):
        raise NotImplementedError

    @abstractmethod
    def motor2joint_vel(self, joint_pos: np.ndarray, motor_vel: np.ndarray):
        raise NotImplementedError

    @abstractmethod
    def joint2motor_torque(self, joint_pos: np.ndarray, joint_torque: np.ndarray):
        raise NotImplementedError

    @abstractmethod
    def motor2joint_torque(self, joint_pos: np.ndarray, motor_torque: np.ndarray):
        raise NotImplementedError

class DirectSolver(BaseSolver):
    def __init__(self, solver_params=None):
        super().__init__(solver_params=solver_params)

        if self.solver_params is not None and self.solver_params.get("negative", False):
            self.oritation = 1
        else:
            self.oritation = -1

    def joint2motor_pos(self, joint_pos: np.ndarray):
        return joint_pos * self.oritation

    def motor2joint_pos(self, motor_pos: np.ndarray):
        return motor_pos * self.oritation

    def joint2motor_vel(self, joint_pos: np.ndarray, joint_vel: np.ndarray):
        return joint_vel * self.oritation

    def motor2joint_vel(self, joint_pos: np.ndarray, motor_vel: np.ndarray):
        return motor_vel * self.oritation

    def joint2motor_torque(self, joint_pos: np.ndarray, joint_torque: np.ndarray):
        return joint_torque * self.oritation

    def motor2joint_torque(self, joint_pos: np.ndarray, motor_torque: np.ndarray):
        return motor_torque * self.oritation
