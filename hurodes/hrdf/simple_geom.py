from dataclasses import dataclass
from re import A
from typing import Union, Type

from numpy import size
import mujoco

from hurodes.hrdf.base.attribute import Position, Quaternion, Name, BodyName, Attribute, SingleFloat, SingleInt
from hurodes.hrdf.base.info import Infos


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
class GeomType(Name):
    name: str = "type"

@dataclass
class Size(Attribute):
    name: str = "size"
    dtype: Union[Type, str] = float
    is_array: bool = True
    dim: int = 3

@dataclass
class RGBA(Attribute):
    """RGBA color attribute for Geom"""
    name: str = "rgba"
    dtype: Union[Type, str] = float
    is_array: bool = True
    dim: int = 4

# List of all Geom attribute classes
ATTR_CLASSES = [
    ConType,
    ConAffinity,
    StaticFriction,
    DynamicFriction,
    Restitution,
    GeomType,
    Size,
    # position attributes
    Position,
    Quaternion,
    # others
    RGBA,
    BodyName,
    Name,
]

GEOM_NAME2ID = {
    "box": mujoco.mjtGeom.mjGEOM_BOX,
    "sphere": mujoco.mjtGeom.mjGEOM_SPHERE,
    "capsule": mujoco.mjtGeom.mjGEOM_CAPSULE,
    "cylinder": mujoco.mjtGeom.mjGEOM_CYLINDER,
    "ellipsoid": mujoco.mjtGeom.mjGEOM_ELLIPSOID,
}

GEOM_ID2NAME = {v: k for k, v in GEOM_NAME2ID.items()}

class SimpleGeomInfos(Infos):
    """Simple Geom class following the InfoList pattern"""
    def __init__(self):
        attrs = [attr_class() for attr_class in ATTR_CLASSES]
        super().__init__(attrs)

    def specific_parse_mujoco(self, info_dict, part_model, part_spec=None, whole_model=None, whole_spec=None):
        info_dict["body_name"] = whole_spec.bodies[int(part_model.bodyid)].name
        info_dict["static_friction"] = part_model.friction[0]
        info_dict["dynamic_friction"] = part_model.friction[0]
        info_dict["restitution"] = None

        assert int(part_model.type) in GEOM_ID2NAME, f"Invalid geom type: {part_model.type}"
        info_dict["type"] = GEOM_ID2NAME[int(part_model.type)]
        return info_dict

    def specific_generate_mujoco(self, mujoco_dict, tag=None):
        del mujoco_dict["body_name"]
        del mujoco_dict["restitution"]

        if mujoco_dict["static_friction"] is not None:
            mujoco_dict["friction"] = f"{mujoco_dict['static_friction']} 0.005 0.0001"
        del mujoco_dict['static_friction']
        del mujoco_dict['dynamic_friction']

        return mujoco_dict
