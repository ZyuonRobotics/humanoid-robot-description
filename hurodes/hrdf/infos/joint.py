from dataclasses import dataclass

from hurodes.hrdf.base.attribute import Position, Axis, Name, AttributeBase, BodyName
from hurodes.hrdf.base.info import InfoBase


@dataclass
class Range(AttributeBase):
    name: str = "range"
    dim: int = 2
    mujoco_name: str = "range"
    urdf_path: tuple = ("limit", ("lower", "upper"))
    default_value: tuple = (-3.14, 3.14)

@dataclass
class Type(AttributeBase):
    name: str = "type"
    urdf_path: tuple = ("type",)
    default_value: str = "revolute"


@dataclass
class Armature(AttributeBase):
    name: str = "armature"
    mujoco_name: str = "armature"
    default_value: float = 0.05

@dataclass
class StitaticFriction(AttributeBase):
    name: str = "static_friction"
    mujoco_name: str = "frictionloss"
    urdf_path: tuple = ("dynamics", "friction")
    default_value: float = 0.05

@dataclass
class DynamicFriction(AttributeBase):
    name: str = "dynamic_friction"
    mujoco_name: str = "frictionloss"
    default_value: float = 0.05

@dataclass
class ViscousFriction(AttributeBase):
    name: str = "viscous_friction"
    mujoco_name: str = "damping"
    urdf_path: tuple = ("dynamics", "damping")
    default_value: float = 0.05

class JointInfo(InfoBase):
    info_name = "JointInfo"
    attr_classes = (
        # joint attributes
        Type,
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
    def _specific_parse_mujoco(cls, info_dict, part_model, part_spec=None, **kwargs):
        whole_spec = kwargs["whole_spec"]
        info_dict["body_name"] = whole_spec.bodies[int(part_model.bodyid)].name.replace("-", "_")
        info_dict["name"] = part_spec.name.replace("-", "_")
        return info_dict

    @classmethod
    def _specific_parse_urdf(cls, info_dict, elem, root_elem, **kwargs):
        assert elem.tag == "joint", f"Expected joint element, got {elem.tag}"

        info_dict["name"] = elem.get("name", "").replace("-", "_")
        info_dict["body_name"] = elem.find("child").get("link").replace("-", "_")
        info_dict["pos"] = [0., 0., 0.]
        info_dict["axis"] = [float(x) for x in info_dict["axis"].split()]
        info_dict["type"] = elem.get("type", "revolute")
        
        return info_dict

    def _specific_generate_mujoco(self, mujoco_dict, extra_dict, tag):
        joint_type = extra_dict["type"].to_string()
        if joint_type == "fixed":
            # For fixed joints, we don't need to set type or limited
            pass
        else:
            mujoco_dict["type"] = "hinge"
            mujoco_dict["limited"] = "true"
        return mujoco_dict

    def _specific_generate_urdf(self, urdf_dict, extra_dict, tag):
        del urdf_dict[('origin', 'xyz')]
        urdf_dict[("type",)] = extra_dict["type"].to_string()
        return urdf_dict
