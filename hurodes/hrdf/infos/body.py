from dataclasses import dataclass
from typing import Union, Type

from hurodes.hrdf.base.attribute import Position, Quaternion, Name, Id, AttributeBase, AttributeBase
from hurodes.hrdf.base.info import InfoBase
from hurodes.utils.convert import str_quat2rpy, str_rpy2quat
from hurodes.utils.xml import extract_attr_from_elem


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
    default_value: float = 1.0

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
    def _specific_parse_mujoco(cls, info_dict, part_model, part_spec=None, **kwargs):
        info_dict["id"] = part_model.id - 1 # skip the world body
        info_dict["name"] = part_model.name.replace("-", "_")
        return info_dict

    @classmethod
    def _specific_parse_urdf(cls, info_dict, elem, root_elem, **kwargs):
        link_nodes = kwargs["link_nodes"]
        link_name = info_dict["name"]
        assert elem.tag == "link", f"Expected link element, got {elem.tag}"

        del info_dict["pos"]
        if link_nodes[link_name].id == 0:
            info_dict["pos"], info_dict["quat"] = [0, 0, 0], [1, 0, 0, 0]
        else:
            info_dict["pos"] = link_nodes[link_name].pos
            info_dict["quat"] = link_nodes[link_name].quat

        info_dict["name"] = link_name.replace("-", "_")
        info_dict["id"] = link_nodes[link_name].id
        

        inertial_rpy = extract_attr_from_elem(elem, ("inertial", "origin", "rpy"))
        info_dict["inertial_quat"] = str_rpy2quat(inertial_rpy).split()
        info_dict["inertial_pos"] = info_dict["inertial_pos"].split()
        return info_dict

    def _specific_generate_mujoco(self, mujoco_dict, extra_dict, tag):
        if tag == "body":
            return {name: mujoco_dict[name] for name in ["name", "pos", "quat"]}
        elif tag == "inertial":
            return {
                "diaginertia": mujoco_dict["inertia"],
                "mass": mujoco_dict["mass"],
                "pos": mujoco_dict["ipos"],
                "quat": mujoco_dict["iquat"]
            }
        else:
            raise ValueError(f"Invalid tag: {tag}")

    def _specific_generate_urdf(self, urdf_dict, extra_dict, tag):
        if tag == "link":
            del urdf_dict[("origin", "xyz")]
            urdf_dict[("inertial", "origin", "rpy")] = str_quat2rpy(extra_dict["inertial_quat"].to_string())
            urdf_dict[("inertial", "inertia", ("ixy", "ixz", "iyz"))] = "0. 0. 0."
            return urdf_dict
        elif tag == "child":
            return {
                ("origin", "xyz"): urdf_dict.pop(("origin", "xyz")),
                ("origin", "rpy"): str_quat2rpy(extra_dict["quat"].to_string()),
                ("child", "link"): extra_dict["name"].data
            }
        elif tag == "parent":
            return {("parent", "link"): extra_dict["name"].data}
        else:
            raise ValueError(f"Invalid tag: {tag}")
