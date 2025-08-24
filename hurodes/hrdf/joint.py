from dataclasses import dataclass

from hurodes.hrdf.base.attribute import Position, Axis, Name, Range, SingleFloat, BodyName
from hurodes.hrdf.base.info import InfoList

@dataclass
class Armature(SingleFloat):
    name: str = "armature"

@dataclass
class StitaticFriction(SingleFloat):
    name: str = "static_friction"
    mujoco_name: str = "frictionloss"

@dataclass
class DynamicFriction(SingleFloat):
    name: str = "dynamic_friction"
    mujoco_name: str = "frictionloss"

@dataclass
class ViscousFriction(SingleFloat):
    name: str = "viscous_friction"
    mujoco_name: str = "damping"

ATTR_CLASSES = [
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
]

class Joint(InfoList):
    def __init__(self):
        attrs = [attr_class() for attr_class in ATTR_CLASSES]

        super().__init__(attrs)

    def specific_parse_mujoco(self, info_dict, part_model, part_spec=None, whole_model=None, whole_spec=None):
        info_dict["body_name"] = whole_spec.bodies[int(part_model.bodyid)].name
        return info_dict

if __name__ == "__main__":
    joint = Joint()
    print(joint.infos)