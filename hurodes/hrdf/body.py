from dataclasses import dataclass
from typing import Union, Type

from hurodes.hrdf.base.attribute import Position, Quaternion, Name, Id, AttributeBase, AttributeBase
from hurodes.hrdf.base.info import InfoBase
from hurodes.utils.convert import str_quat2rpy


@dataclass
class DiagInertia(AttributeBase):
    """Diagonal inertia matrix, expressing the body inertia relative to the inertial frame.
    """
    name: str = "diag_inertia"
    mujoco_name: str = "inertia"
    urdf_path: tuple = ("inertial", "inertia", ("ixx", "iyy", "izz"))
    dim: int = 3

@dataclass
class Mass(AttributeBase):
    name: str = "mass"
    mujoco_name: str = "mass"
    urdf_path: tuple = ("inertial", "mass", "value")

@dataclass
class InertialPosition(Position):
    """Position of the inertial frame. 
    """
    name: str = "inertial_pos"
    mujoco_name: str = "ipos"
    urdf_path: tuple = ("inertial", "origin", "xyz")

@dataclass
class InertialQuaternion(Quaternion):
    """Quaternion of the inertial frame.
    """
    name: str = "inertial_quat"
    mujoco_name: str = "iquat"

class BodyInfo(InfoBase):
    info_name = "BodyInfo"
    attr_classes = (
        # body attributes
        DiagInertia,
        Mass,
        InertialPosition,
        InertialQuaternion,
        # body position
        Position,
        Quaternion,
        # others
        Name,
        Id,
    )

    @classmethod
    def _specific_parse_mujoco(cls, info_dict, part_model, part_spec=None, whole_model=None, whole_spec=None):
        info_dict["id"] = part_model.id - 1 # skip the world body
        info_dict["name"] = part_model.name.replace("-", "_")
        return info_dict

    def _specific_generate_mujoco(self, mujoco_dict, extra_dict, tag):
        if tag == "body":
            mujoco_dict = {name: mujoco_dict[name] for name in ["name", "pos", "quat"]}
        elif tag == "inertial":
            for name in ["name", "pos", "quat", "id"]:
                del mujoco_dict[name]
            mujoco_dict["diaginertia"] = mujoco_dict.pop("inertia")
            mujoco_dict["pos"] = mujoco_dict.pop("ipos")
            mujoco_dict["quat"] = mujoco_dict.pop("iquat")
        else:
            raise ValueError(f"Invalid tag: {tag}")
        return mujoco_dict

    def _specific_generate_urdf(self, urdf_dict, extra_dict, tag):
        if tag == "link":
            del urdf_dict[("origin", "xyz")]
            urdf_dict[("inertial", "origin", "rpy")] = str_quat2rpy(extra_dict["inertial_quat"])
            return urdf_dict
        elif tag == "joint":
            return {
                ("origin", "xyz"): urdf_dict.pop(("origin", "xyz")),
                ("origin", "rpy"): str_quat2rpy(extra_dict["quat"])
            }
        else:
            raise ValueError(f"Invalid tag: {tag}")
