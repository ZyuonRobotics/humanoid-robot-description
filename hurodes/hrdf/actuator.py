from dataclasses import dataclass

from hurodes.hrdf.base.attribute import Name, SingleFloat
from hurodes.hrdf.base.info import Infos

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

class ActuatorInfos(Infos):
    def __init__(self):
        attrs = [attr_class() for attr_class in ATTR_CLASSES]

        super().__init__(attrs)

    def specific_parse_mujoco(self, info_dict, part_model, part_spec=None, whole_model=None, whole_spec=None):
        assert part_model.ctrlrange[0] == - part_model.ctrlrange[1]
        info_dict["peak_torque"] = part_model.ctrlrange[1]
        info_dict["peak_velocity"] = None
        info_dict["damping"] = None
        info_dict["stiffness"] = None
        info_dict["joint_name"] = part_spec.target
        return info_dict

    def specific_generate_mujoco(self, mujoco_dict, tag=None):
        del mujoco_dict["damping"]
        del mujoco_dict["stiffness"]
        del mujoco_dict["peak_velocity"]

        mujoco_dict["joint"] = mujoco_dict.pop("joint_name")  
        mujoco_dict["ctrlrange"] = f"-{mujoco_dict['peak_torque']} {mujoco_dict['peak_torque']}"
        del mujoco_dict["peak_torque"]
        return mujoco_dict
