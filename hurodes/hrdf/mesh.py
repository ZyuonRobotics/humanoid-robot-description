from hurodes.hrdf.base.attribute import Position, Quaternion, Name, BodyName
from hurodes.hrdf.simple_geom import ConType, ConAffinity, RGBA, StaticFriction, DynamicFriction, Restitution
from hurodes.hrdf.base.info import InfoBase

class MeshInfo(InfoBase):
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
    def specific_parse_mujoco(cls, info_dict, part_model, part_spec=None, whole_model=None, whole_spec=None):
        info_dict["body_name"] = whole_spec.bodies[int(part_model.bodyid)].name.replace("-", "_")
        info_dict["name"] = part_spec.meshname.replace("-", "_")

        # idk why, but the value from part_model(_MjModelGeomViews) is wrong
        info_dict["pos"] = part_spec.pos
        info_dict["quat"] = part_spec.quat

        info_dict["static_friction"] = part_model.friction[0]
        info_dict["dynamic_friction"] = part_model.friction[0]
        info_dict["restitution"] = None
        return info_dict

    def specific_generate_mujoco(self, mujoco_dict, tag=None):
        del mujoco_dict["body_name"]
        del mujoco_dict["restitution"]
        mujoco_dict["mesh"] = mujoco_dict.pop("name")
        mujoco_dict["type"] = "mesh"

        if mujoco_dict["static_friction"] is not None:
            mujoco_dict["friction"] = f"{mujoco_dict['static_friction']} 0.005 0.0001"
        del mujoco_dict['static_friction']
        del mujoco_dict['dynamic_friction']
        return mujoco_dict
