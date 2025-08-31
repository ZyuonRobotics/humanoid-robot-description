from dataclasses import dataclass

from hurodes.hrdf.base.attribute import Name, AttributeBase, JointName
from hurodes.hrdf.base.info import InfoBase

@dataclass
class PeakTorque(AttributeBase):
    name: str = "peak_torque"
    urdf_path: tuple = ("limit", "effort")

@dataclass
class PeakVelocity(AttributeBase):
    name: str = "peak_velocity"
    urdf_path: tuple = ("limit", "velocity")

@dataclass
class DGain(AttributeBase):
    name: str = "d_gain"

@dataclass
class PGain(AttributeBase):
    name: str = "p_gain"

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
    def _specific_parse_mujoco(cls, info_dict, part_model, part_spec=None, **kwargs):
        assert part_model.ctrlrange[0] == - part_model.ctrlrange[1]
        info_dict["peak_torque"] = part_model.ctrlrange[1]
        info_dict["joint_name"] = part_spec.target.replace("-", "_")
        info_dict["name"] = part_model.name.replace("-", "_")
        return info_dict

    @classmethod
    def _specific_parse_urdf(cls, info_dict, elem, root_elem, **kwargs):
        assert elem.tag == "joint", f"Expected joint element, got {elem.tag}"
        
        joint_name = elem.get("name", "").replace("-", "_")
        info_dict["joint_name"] = joint_name
        info_dict["name"] = f"{joint_name}_motor"

        return info_dict

    def _specific_generate_mujoco(self, mujoco_dict, extra_dict, tag):
        mujoco_dict["ctrlrange"] = f"-{extra_dict['peak_torque']} {extra_dict['peak_torque']}"
        mujoco_dict["joint"] = extra_dict["joint_name"]
        mujoco_dict["ctrllimited"] = "true"
        return mujoco_dict

    def _specific_generate_urdf(self, urdf_dict, extra_dict, tag):
        del urdf_dict[('name',)]
        return urdf_dict
