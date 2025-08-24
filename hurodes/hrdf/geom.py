from dataclasses import dataclass
from typing import Union, Type

from hurodes.hrdf.base.attribute import Position, Quaternion, Name, BodyName, Attribute, SingleFloat, SingleInt
from hurodes.hrdf.base.info import InfoList


@dataclass
class StaticFriction(SingleFloat):
    """Static friction attribute for Geom"""
    name: str = "static_friction"

@dataclass
class DynamicFriction(SingleFloat):
    """Dynamic friction attribute for Geom"""
    name: str = "dynamic_friction"

@dataclass
class Restitution(SingleFloat):
    """Restitution attribute for Geom"""
    name: str = "restitution"

@dataclass
class ConType(SingleInt):
    """Contact type attribute for Geom"""
    name: str = "contype"

@dataclass
class ConAffinity(SingleInt):
    """Contact affinity attribute for Geom"""
    name: str = "conaffinity"

@dataclass
class RGBA(Attribute):
    """RGBA color attribute for Geom"""
    name: str = "rgba"
    dtype: Union[Type, str] = float
    is_array: bool = True
    dim: int = 4

# List of all Geom attribute classes
ATTR_CLASSES = [
    # contact attributes
    ConType,
    ConAffinity,
    # position attributes
    Position,
    Quaternion,
    # others
    RGBA,
    BodyName,
    Name,
]

class Geom(InfoList):
    """Geom class following the InfoList pattern"""
    def __init__(self):
        attrs = [attr_class() for attr_class in ATTR_CLASSES]
        super().__init__(attrs)

    def specific_parse_mujoco(self, info_dict, part_model, part_spec=None, whole_model=None, whole_spec=None):
        info_dict["body_name"] = whole_spec.bodies[int(part_model.bodyid)].name
        info_dict["static_friction"] = part_model.friction[0]
        info_dict["dynamic_friction"] = part_model.friction[0]
        info_dict["restitution"] = None
        return info_dict

if __name__ == "__main__":
    # Test the Geom class
    Geom = Geom()
    print("Geom attributes:")
    for name, info in Geom.infos.items():
        print(f"  {name}: {info.dtype.__name__} {'(array)' if info.is_array else '(single)'} dim={info.dim}")
