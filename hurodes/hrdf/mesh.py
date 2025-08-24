from hurodes.hrdf.base.attribute import Position, Quaternion, Name, BodyName
from hurodes.hrdf.simple_geom import ConType, ConAffinity, RGBA
from hurodes.hrdf.base.info import Infos

# List of all mesh attribute classes
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

class MeshInfos(Infos):
    """Mesh class following the InfoList pattern"""
    def __init__(self):
        attrs = [attr_class() for attr_class in ATTR_CLASSES]
        super().__init__(attrs)

    def specific_parse_mujoco(self, info_dict, part_model, part_spec=None, whole_model=None, whole_spec=None):
        info_dict["body_name"] = whole_spec.bodies[int(part_model.bodyid)].name
        info_dict["name"] = part_spec.meshname
        return info_dict
