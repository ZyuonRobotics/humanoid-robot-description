from dataclasses import dataclass

from hurodes.hrdf.base.attribute import Position, Axis, Name, Range, SingleFloat, BodyName
from hurodes.hrdf.base.info import InfoBase

@dataclass
class Armature(SingleFloat):
    name: str = "armature"
    urdf_path: tuple = None
@dataclass
class StitaticFriction(SingleFloat):
    name: str = "static_friction"
    mujoco_name: str = "frictionloss"
    urdf_path: tuple = ("dynamics", "friction")

@dataclass
class DynamicFriction(SingleFloat):
    name: str = "dynamic_friction"
    mujoco_name: str = "frictionloss"
    urdf_path: tuple = None

@dataclass
class ViscousFriction(SingleFloat):
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
    def specific_parse_mujoco(cls, info_dict, part_model, part_spec=None, whole_model=None, whole_spec=None):
        info_dict["body_name"] = whole_spec.bodies[int(part_model.bodyid)].name
        return info_dict

    def specific_generate_mujoco(self, mujoco_dict, tag=None):
        del mujoco_dict["body_name"]
        mujoco_dict["type"] = "hinge"
        return mujoco_dict
