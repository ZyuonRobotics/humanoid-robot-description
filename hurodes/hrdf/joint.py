from dataclasses import dataclass
from typing import Union, Type
from hurodes.hrdf.base.attribute import Position, Axis, Name, Id, Range, SingleFloat
from hurodes.hrdf.base.info import InfoList

@dataclass
class BodyId(Id):
    name: str = "body_id"

@dataclass
class Armature(SingleFloat):
    name: str = "armature"

@dataclass
class StitaticFriction(SingleFloat):
    name: str = "static_friction"

@dataclass
class DynamicFriction(SingleFloat):
    name: str = "dynamic_friction"


@dataclass
class ViscousFriction(SingleFloat):
    name: str = "viscous_friction"


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
    BodyId,
    Range,
]

class Joint(InfoList):
    def __init__(self):
        attrs = [attr_class() for attr_class in ATTR_CLASSES]

        super().__init__(attrs)

if __name__ == "__main__":
    joint = Joint()
    print(joint.infos)