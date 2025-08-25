from dataclasses import dataclass
from typing import Union, Type

from hurodes.hrdf.base.attribute import Position, Quaternion, Name, Id, SingleFloat, Attribute
from hurodes.hrdf.base.info import Infos

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
    mujoco_name: str = "ipos"

@dataclass
class InertiaQuaternion(Quaternion):
    name: str = "inertia_quaternion"
    mujoco_name: str = "iquat"

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

class BodyInfos(Infos):
    def __init__(self):
        attrs = [attr_class() for attr_class in ATTR_CLASSES]

        super().__init__(attrs)

    def specific_parse_mujoco(self, info_dict, part_model, part_spec=None, whole_model=None, whole_spec=None):
        info_dict["id"] = part_model.id - 1 # skip the world body
        return info_dict

    def specific_generate_mujoco(self, mujoco_dict, tag=None):
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
