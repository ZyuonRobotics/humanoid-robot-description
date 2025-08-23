from dataclasses import dataclass
from typing import Union, Type
from hurodes.hrdf.base.attribute import Position, Quaternion, Axis, Name, Id, Range, SingleFloat, Attribute
from hurodes.hrdf.base.info import InfoList

@dataclass
class Inertia(Attribute):
    name: str = "inertia"
    dtype: Union[Type, str] = float
    is_array: bool = True
    dim: int = 3

@dataclass
class Mass(SingleFloat):
    name: str = "mass"

@dataclass
class InertiaPosition(Position):
    name: str = "inertia_position"

@dataclass
class InertiaQuaternion(Quaternion):
    name: str = "inertia_quaternion"

ATTR_CLASSES = [
    # body attributes
    Inertia,
    Mass,
    InertiaPosition,
    InertiaQuaternion,
    # body position
    Position,
    Quaternion,
    # others
    Name,
    Id,
]

class Body(InfoList):
    def __init__(self):
        attrs = [attr_class() for attr_class in ATTR_CLASSES]

        super().__init__(attrs)

if __name__ == "__main__":
    body = Body()
    print(body.infos)