from dataclasses import dataclass
from typing import Union, Type
from hurodes.hrdf.base.attribute import Position, Quaternion, Name, Id, Attribute, SingleFloat
from hurodes.hrdf.base.info import InfoList

@dataclass
class BodyId(Id):
    """Body ID attribute for mesh"""
    name: str = "bodyid"

@dataclass
class ConType(Attribute):
    """Contact type attribute for mesh"""
    name: str = "contype"
    dtype: Union[Type, str] = int
    is_array: bool = False
    dim: int = 0

@dataclass
class ConAffinity(Attribute):
    """Contact affinity attribute for mesh"""
    name: str = "conaffinity"
    dtype: Union[Type, str] = int
    is_array: bool = False
    dim: int = 0

@dataclass
class MeshName(Name):
    """Mesh name attribute"""
    name: str = "mesh"

@dataclass
class RGBA(Attribute):
    """RGBA color attribute for mesh"""
    name: str = "rgba"
    dtype: Union[Type, str] = float
    is_array: bool = True
    dim: int = 4

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
    BodyId,
    MeshName,
]

class Mesh(InfoList):
    """Mesh class following the InfoList pattern"""
    def __init__(self):
        attrs = [attr_class() for attr_class in ATTR_CLASSES]
        super().__init__(attrs)

if __name__ == "__main__":
    # Test the mesh class
    mesh = Mesh()
    print("Mesh attributes:")
    for name, info in mesh.infos.items():
        print(f"  {name}: {info.dtype.__name__} {'(array)' if info.is_array else '(single)'} dim={info.dim}")
