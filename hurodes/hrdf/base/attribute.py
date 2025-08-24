from typing import Union, Type
from dataclasses import dataclass, field

@dataclass
class Attribute:
    name: str
    dtype: Union[Type, str]
    is_array: bool
    dim: int
    mujoco_name: str = field(default="")
    
    def __post_init__(self):
        # If mujoco_name is not specified, use name as default
        if not self.mujoco_name:
            self.mujoco_name = self.name


@dataclass
class SingleFloat(Attribute):
    dtype: Union[Type, str] = float
    is_array: bool = False
    dim: int = 0

@dataclass
class SingleInt(Attribute):
    dtype: Union[Type, str] = int
    is_array: bool = False
    dim: int = 0
@dataclass
class Position(Attribute):
    name: str = "pos"
    dtype: Union[Type, str] = float
    is_array: bool = True
    dim: int = 3

@dataclass
class Quaternion(Attribute):
    name: str = "quat"
    dtype: Union[Type, str] = float
    is_array: bool = True
    dim: int = 4

@dataclass
class Axis(Attribute):
    name: str = "axis"
    dtype: Union[Type, str] = float
    is_array: bool = True
    dim: int = 3

@dataclass
class Name(Attribute):
    name: str = "name"
    dtype: Union[Type, str] = str
    is_array: bool = False
    dim: int = 0

@dataclass
class BodyName(Name):
    name: str = "body_name"

@dataclass
class Id(SingleInt):
    name: str = "id"
    dtype: Union[Type, str] = int
    is_array: bool = False
    dim: int = 0

@dataclass
class Inertia(Attribute):
    name: str = "inertia"
    dtype: Union[Type, str] = float
    is_array: bool = True
    dim: int = 3

@dataclass
class Range(Attribute):
    name: str = "range"
    dtype: Union[Type, str] = float
    is_array: bool = True
    dim: int = 2

