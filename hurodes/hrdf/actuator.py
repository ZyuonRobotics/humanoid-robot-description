from dataclasses import dataclass
from typing import Union, Type
from hurodes.hrdf.base.attribute import Position, Axis, Name, Id, Range, SingleFloat
from hurodes.hrdf.base.info import InfoList

@dataclass
class PeakTorque(SingleFloat):
    name: str = "peak_torque"

@dataclass
class PeakVelocity(SingleFloat):
    name: str = "peak_velocity"

@dataclass
class PeakVelocity(SingleFloat):
    name: str = "peak_velocity"

@dataclass
class Damping(SingleFloat):
    name: str = "damping"

@dataclass
class Stiffness(SingleFloat):
    name: str = "stiffness"

@dataclass
class JointName(Name):
    name: str = "joint_name"

ATTR_CLASSES = [
    PeakTorque,
    PeakVelocity,
    Damping,
    Stiffness,
    JointName,
    Name,

]

class Actuator(InfoList):
    def __init__(self):
        attrs = [attr_class() for attr_class in ATTR_CLASSES]

        super().__init__(attrs)

if __name__ == "__main__":
    actuator = Actuator()
    print(actuator.infos)