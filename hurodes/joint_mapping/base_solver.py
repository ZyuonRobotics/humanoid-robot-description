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

    def joint2motor(self, joint_pos, joint_vel, joint_torque):
        motor_pos = self.joint2motor_pos(joint_pos)
        motor_vel = self.joint2motor_vel(joint_pos, joint_vel)
        motor_torque = self.joint2motor_torque(joint_pos, joint_torque)
        return motor_pos, motor_vel, motor_torque

    @abstractmethod
    def motor2joint_vel(self, joint_pos: np.ndarray, motor_vel: np.ndarray):
        raise NotImplementedError

    @abstractmethod
    def joint2motor_torque(self, joint_pos: np.ndarray, joint_torque: np.ndarray):
        raise NotImplementedError

    @abstractmethod
    def motor2joint_torque(self, joint_pos: np.ndarray, motor_torque: np.ndarray):
        raise NotImplementedError

    def motor2joint(self, motor_pos, motor_vel, motor_torque):
        joint_pos = self.motor2joint_pos(motor_pos)
        joint_vel = self.motor2joint_vel(joint_pos, motor_vel)
        joint_torque = self.motor2joint_torque(joint_pos, motor_torque)
        return joint_pos, joint_vel, joint_torque
