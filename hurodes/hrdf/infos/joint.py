from dataclasses import dataclass

from hurodes.hrdf.base.attribute import Position, Axis, Name, AttributeBase, BodyName
from hurodes.hrdf.base.info import InfoBase



@dataclass
class Range(AttributeBase):
    name: str = "range"
    dim: int = 2
    mujoco_name: str = "range"
    urdf_path: tuple = ("limit", ("lower", "upper"))

@dataclass
class Armature(AttributeBase):
    name: str = "armature"
    mujoco_name: str = "armature"

@dataclass
class StitaticFriction(AttributeBase):
    name: str = "static_friction"
    mujoco_name: str = "frictionloss"
    urdf_path: tuple = ("dynamics", "friction")

@dataclass
class DynamicFriction(AttributeBase):
    name: str = "dynamic_friction"
    mujoco_name: str = "frictionloss"

@dataclass
class ViscousFriction(AttributeBase):
    name: str = "viscous_friction"
    mujoco_name: str = "damping"
    urdf_path: tuple = ("dynamics", "damping")

class JointInfo(InfoBase):
    info_name = "JointInfo"
    attr_classes = (
        # joint attributes
        Armature,
        StitaticFriction,
        DynamicFriction,
        ViscousFriction,
        # joint position
        Position,
        Axis,
        # others
        Name,
        BodyName,
        Range,
    )

    @classmethod
    def _specific_parse_mujoco(cls, info_dict, part_model, part_spec=None, whole_model=None, whole_spec=None):
        info_dict["body_name"] = whole_spec.bodies[int(part_model.bodyid)].name.replace("-", "_")
        info_dict["name"] = part_spec.name.replace("-", "_")
        return info_dict

    def _specific_generate_mujoco(self, mujoco_dict, extra_dict, tag):
        mujoco_dict["type"] = "hinge"
        mujoco_dict["limited"] = "true"
        return mujoco_dict

    def _specific_generate_urdf(self, urdf_dict, extra_dict, tag):
        del urdf_dict[('origin', 'xyz')]
        urdf_dict[("type",)] = "revolute"
        return urdf_dict
