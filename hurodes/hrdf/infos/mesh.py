import xml.etree.ElementTree as ET
from pathlib import Path

from hurodes.hrdf.base.attribute import Position, Quaternion, Name, BodyName
from hurodes.hrdf.base.info import extract_attr_from_elem
from hurodes.hrdf.infos.simple_geom import SimpleGeomInfo, ContactType, ContactAffinity, RGBA, StaticFriction, DynamicFriction, Restitution
from hurodes.utils.convert import str_quat2rpy, str_rpy2quat

class MeshInfo(SimpleGeomInfo):
    info_name = "MeshInfo"
    attr_classes = (
        # contact attributes
        ContactType,
        ContactAffinity,
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
    def _specific_parse_mujoco(cls, info_dict, part_model, part_spec=None, **kwargs):
        whole_spec = kwargs["whole_spec"]
        info_dict["body_name"] = whole_spec.bodies[int(part_model.bodyid)].name.replace("-", "_")
        info_dict["name"] = part_spec.meshname.replace("-", "_")

        # idk why, but the value from part_model(_MjModelGeomViews) is wrong
        info_dict["pos"] = part_spec.pos
        info_dict["quat"] = part_spec.quat

        info_dict["static_friction"] = part_model.friction[0]
        info_dict["dynamic_friction"] = part_model.friction[0]
        info_dict["restitution"] = None
        return info_dict

    @classmethod
    def _specific_parse_urdf(cls, info_dict, elem, root_elem, **kwargs):
        assert elem.tag in ["visual", "collision"], f"Expected link or joint element, got {elem.tag}"
        body_name = kwargs["body_name"]

        if elem.tag == "visual":
            info_dict["contact_type"], info_dict["contact_affinity"] = 0, 0
        else:
            info_dict["contact_type"], info_dict["contact_affinity"] = 1, 1

        filename = extract_attr_from_elem(elem, ("geometry", "mesh", "filename"))
        info_dict["name"] = filename.split("/")[-1].split(".")[0].replace("-", "_")

        rpy = extract_attr_from_elem(elem, ("origin", "rpy"))
        info_dict["quat"] = [float(x) for x in str_rpy2quat(rpy).split()]
        info_dict["pos"] = [float(x) for x in info_dict["pos"].split()]
        
        if "rgba" in info_dict and info_dict["rgba"] is not None:
            info_dict["rgba"] = [float(x) for x in info_dict["rgba"].split()]

        info_dict["body_name"] = body_name
        return info_dict

    def _specific_generate_mujoco(self, mujoco_dict, extra_dict, tag):
        mujoco_dict["mesh"] = mujoco_dict.pop("name")
        mujoco_dict["type"] = "mesh"

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
        urdf_dict[("geometry", "mesh", "filename")] = extra_dict["name"].to_string() + ".stl"
        return urdf_dict
