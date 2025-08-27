from dataclasses import dataclass

from hurodes.hrdf.base.attribute import Name, SingleFloat, JointName
from hurodes.hrdf.base.info import InfoBase

@dataclass
class PeakTorque(SingleFloat):
    name: str = "peak_torque"
    urdf_path: tuple = ("limit", "effort")

@dataclass
class PeakVelocity(SingleFloat):
    name: str = "peak_velocity"
    mujoco_name: str = None
    urdf_path: tuple = ("limit", "velocity")

@dataclass
class DGain(SingleFloat):
    name: str = "d_gain"
    mujoco_name: str = None
    urdf_path: tuple = None

@dataclass
class PGain(SingleFloat):
    name: str = "p_gain"
    mujoco_name: str = None
    urdf_path: tuple = None

class ActuatorInfo(InfoBase):
    info_name = "ActuatorInfo"
    attr_classes = (
        PeakTorque,
        PeakVelocity,
        DGain,
        PGain,
        JointName,
        Name,
    )

    @classmethod
    def specific_parse_mujoco(cls, info_dict, part_model, part_spec=None, whole_model=None, whole_spec=None):
        assert part_model.ctrlrange[0] == - part_model.ctrlrange[1]
        info_dict["peak_torque"] = part_model.ctrlrange[1]
        info_dict["joint_name"] = part_spec.target
        return info_dict

    def specific_generate_mujoco(self, mujoco_dict, tag=None):
        mujoco_dict["joint"] = mujoco_dict.pop("joint_name")  
        mujoco_dict["ctrlrange"] = f"-{mujoco_dict['peak_torque']} {mujoco_dict['peak_torque']}"
        del mujoco_dict["peak_torque"]
        return mujoco_dict
