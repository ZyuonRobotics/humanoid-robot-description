import xml.etree.ElementTree as ET

from hurodes.hrdf.base.attribute import Position, Quaternion, Name, BodyName
from hurodes.hrdf.infos.simple_geom import SimpleGeomInfo, ConType, ConAffinity, RGBA, StaticFriction, DynamicFriction, Restitution
from hurodes.utils.convert import str_quat2rpy

class MeshInfo(SimpleGeomInfo):
    info_name = "MeshInfo"
    attr_classes = (
        # contact attributes
        ConType,
        ConAffinity,
        StaticFriction,
        DynamicFriction,
        Restitution,
        # position attributes
        Position,
        Quaternion,
        # others
        RGBA,
        BodyName,
        Name,
    )

    @classmethod
    def _specific_parse_mujoco(cls, info_dict, part_model, part_spec=None, whole_model=None, whole_spec=None):
        info_dict["body_name"] = whole_spec.bodies[int(part_model.bodyid)].name.replace("-", "_")
        info_dict["name"] = part_spec.meshname.replace("-", "_")

        # idk why, but the value from part_model(_MjModelGeomViews) is wrong
        info_dict["pos"] = part_spec.pos
        info_dict["quat"] = part_spec.quat

        info_dict["static_friction"] = part_model.friction[0]
        info_dict["dynamic_friction"] = part_model.friction[0]
        info_dict["restitution"] = None
        return info_dict

    def _specific_generate_mujoco(self, mujoco_dict, extra_dict, tag):
        mujoco_dict["mesh"] = mujoco_dict.pop("name")
        mujoco_dict["type"] = "mesh"

        friction = extra_dict.get("dynamic_friction", extra_dict.get("static_friction", 1.))
        mujoco_dict["friction"] = f"{friction} 0.005 0.0001"
        return mujoco_dict

    def _specific_generate_urdf(self, urdf_dict, extra_dict, tag):
        urdf_dict[("origin", "rpy")] = str_quat2rpy(extra_dict["quat"])
        urdf_dict[("geometry", "mesh", "filename")] = extra_dict["name"] + ".stl"
        return urdf_dict
