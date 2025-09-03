from dataclasses import dataclass

import xml.etree.ElementTree as ET
import mujoco

from hurodes.hrdf.base.attribute import Position, Quaternion, Name, BodyName, AttributeBase
from hurodes.hrdf.base.info import InfoBase, add_attr_to_elem
from hurodes.utils.convert import str_quat2rpy, rpy2quat


@dataclass
class StaticFriction(AttributeBase):
    """Static friction attribute for Geom"""
    name: str = "static_friction"
    default_value: float = 1.

@dataclass
class DynamicFriction(AttributeBase):
    """Dynamic friction attribute for Geom"""
    name: str = "dynamic_friction"
    default_value: float = 1.

@dataclass
class Restitution(AttributeBase):
    """Restitution attribute for Geom"""
    name: str = "restitution"
    default_value: float = 1.0

@dataclass
class ContactType(AttributeBase):
    """Contact type attribute for Geom"""
    name: str = "contact_type"
    dtype: str = "int"
    mujoco_name: str = "contype"
    default_value: int = 1

@dataclass
class ContactAffinity(AttributeBase):
    """Contact affinity attribute for Geom"""
    name: str = "contact_affinity"
    dtype: str = "int"
    mujoco_name: str = "conaffinity"
    default_value: int = 1

@dataclass
class GeomType(AttributeBase):
    name: str = "type"
    dtype: str = "str"
    mujoco_name: str = "type"

@dataclass
class Size(AttributeBase):
    name: str = "size"
    dim: int = 3
    mujoco_name: str = "size"

@dataclass
class RGBA(AttributeBase):
    """RGBA color attribute for Geom"""
    name: str = "rgba"
    dim: int = 4
    mujoco_name: str = "rgba"
    urdf_path: tuple = ("material", "color", "rgba")
    default_value: tuple = (0.8, 0.8, 0.8, 1.0)


GEOM_NAME2ID = {
    "box": mujoco.mjtGeom.mjGEOM_BOX,
    "sphere": mujoco.mjtGeom.mjGEOM_SPHERE,
    "capsule": mujoco.mjtGeom.mjGEOM_CAPSULE,
    "cylinder": mujoco.mjtGeom.mjGEOM_CYLINDER,
    "ellipsoid": mujoco.mjtGeom.mjGEOM_ELLIPSOID,
}

GEOM_ID2NAME = {v: k for k, v in GEOM_NAME2ID.items()}

class SimpleGeomInfo(InfoBase):
    info_name = "SimpleGeomInfo"
    attr_classes = (
        ContactType,
        ContactAffinity,
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
    )

    @classmethod
    def _specific_parse_mujoco(cls, info_dict, part_model, part_spec=None, **kwargs):
        whole_spec = kwargs["whole_spec"]
        info_dict["body_name"] = whole_spec.bodies[int(part_model.bodyid)].name.replace("-", "_")
        info_dict["name"] = part_spec.name.replace("-", "_")
        info_dict["static_friction"] = part_model.friction[0]
        info_dict["dynamic_friction"] = part_model.friction[0]
        info_dict["restitution"] = None

        assert int(part_model.type) in GEOM_ID2NAME, f"Invalid geom type: {part_model.type}"
        info_dict["type"] = GEOM_ID2NAME[int(part_model.type)]
        return info_dict

    @classmethod
    def _specific_parse_urdf(cls, info_dict, elem, root_elem, **kwargs):
        assert elem.tag in ["visual", "collision"], f"Expected visual or collision element, got {elem.tag}"
        
        import pdb
        pdb.set_trace()

        return info_dict

    def _specific_generate_mujoco(self, mujoco_dict, extra_dict, tag):
        if mujoco_dict["name"] == "nan":
            del mujoco_dict["name"]

        if "static_friction" in extra_dict and extra_dict["static_friction"].data is not None:
            friction = extra_dict["static_friction"].data
        elif "dynamic_friction" in extra_dict and extra_dict["dynamic_friction"].data is not None:
            friction = extra_dict["dynamic_friction"].data
        else:
            friction = 1.0
        mujoco_dict["friction"] = f"{friction} 0.005 0.0001"
        return mujoco_dict

    def _specific_generate_urdf(self, urdf_dict, extra_dict, tag):
        urdf_dict[("origin", "rpy")] = str_quat2rpy(extra_dict["quat"].to_string())
        return urdf_dict
    

    def to_urdf_elem(self, root_elem, tag=None):
        urdf_dict, extra_dict = self._to_urdf_dict(tag)

        if float(extra_dict["contact_type"]) == float(extra_dict["contact_affinity"]) == 0.0:
            sub_elem = ET.SubElement(root_elem, "visual")
        elif float(extra_dict["contact_type"]) == float(extra_dict["contact_affinity"]) == 1.0:
            sub_elem = ET.SubElement(root_elem, "collision")
        else:
            raise ValueError(f"Invalid contact_type and contact_affinity: {extra_dict['contact_type']}, {extra_dict['contact_affinity']}")

        for attr_path, attr_value in urdf_dict.items():
            add_attr_to_elem(sub_elem, attr_path, attr_value)
        return sub_elem
