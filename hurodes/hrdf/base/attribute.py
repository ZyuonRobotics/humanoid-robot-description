from typing import Union, Type
from dataclasses import dataclass, field

@dataclass
class Attribute:
    name: str
    dtype: Union[Type, str]
    is_array: bool
    dim: int

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
class Id(Attribute):
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


@dataclass
class SingleFloat(Attribute):
    dtype: Union[Type, str] = float
    is_array: bool = False
    dim: int = 0