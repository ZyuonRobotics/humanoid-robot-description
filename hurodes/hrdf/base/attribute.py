from typing import Union, Type
from dataclasses import dataclass, field

from typing import Type, Dict, Any, Optional, Union
import numpy as np
from dataclasses import dataclass, field

from bidict import bidict

TYPE_MAP = bidict({
    'int': int,
    'float': float,
    'str': str,
    'bool': bool
})

@dataclass
class AttributeBase:
    """
    Attributes:
        name: Name of the attribute.
        dtype: Data type of the attribute.
        is_array: Whether the attribute is an array.
        dim: Dimension of the attribute.
        mujoco_name: Name of the attribute in Mujoco, if empty string, use name as default. If None, do not generate mujoco attribute.
        urdf_path: Path to the attribute in URDF, if empty tuple, use name as default. If None, do not generate urdf attribute.
    """
    name: str = field(default="")
    dtype: Union[Type, str] = field(default=float)
    is_array: bool = field(default=False)
    dim: int = field(default=0)
    mujoco_name: str = field(default="")
    urdf_path: tuple = field(default=())

    def __init_subclass__(cls):
        if isinstance(cls.dtype, str):
            assert cls.dtype in TYPE_MAP, f"Invalid data type: {cls.dtype}"
            cls.dtype = TYPE_MAP[cls.dtype]
        else:
            assert cls.dtype in TYPE_MAP.values(), f"Invalid data type: {cls.dtype}"

        if cls.is_array:
            assert cls.dim > 1, "Array must have dimension > 1"
        else:
            assert cls.dim == 0
            
        if cls.mujoco_name == "":
            cls.mujoco_name = cls.name
        if cls.urdf_path == ():
            cls.urdf_path = (cls.name,)

    def __post_init__(self):
        self._data = None

    @property
    def data(self) -> Any:
        return self._data

    @data.setter
    def data(self, data: Optional[Union[int, str, float, bool, np.ndarray, list]]):
        if data is None:
            pass
        elif not self.is_array:
            data = self.dtype(data)
        else:
            if isinstance(data, list):
                data = np.array(data)
            
            assert isinstance(data, np.ndarray), f"Invalid data type: {type(data)}, expected: np.ndarray"
            assert data.shape == (self.dim,), f"Invalid data shape: {data.shape}, expected: {self.dim}"
            data = data.astype(self.dtype) if self.dtype != str else data
        self._data = data
    
    # def parse_string(self, string: str):
    #     if not self.is_array:
    #         data = self.type_convert(string)
    #     else:
    #         data = string.split()
    #         assert len(data) == self.dim, f"Expected {self.dim} elements, but got {len(data)}"
    #         data = [self.type_convert(elem) for elem in data]
    #         if self.dtype != str:
    #             data = np.array(data)

    #     self._data = data

    def parse_flat_dict(self, flat_dict: Dict[str, Any]):
        """
        Parse flat dictionary to attribute data. This function assumes that the value must be in the flat_dict.
        """
        assert flat_dict is not None and isinstance(flat_dict, dict), "flat_dict must be a dictionary"
        
        if not self.is_array:
            assert self.name in flat_dict, f"Attribute {self.name} not found in flat_dict"
            self.data = flat_dict[self.name]
        else:
            for i in range(self.dim):
                assert f"{self.name}{i}" in flat_dict, f"Attribute {self.name}{i} not found in attr_dict"
            self.data = [flat_dict[f"{self.name}{i}"] for i in range(self.dim)]

    @classmethod
    def from_flat_dict(cls, flat_dict: Dict[str, Any]):
        info = cls()
        info.parse_flat_dict(flat_dict)
        return info

    @classmethod
    def from_data(cls, data: Any):
        info = cls()
        info.data = data
        return info

    
    def to_dict(self) -> Dict[str, Any]:
        if self.is_array:
            if self.data is None:
                return {f"{self.name}{i}": None for i in range(self.dim)}
            else:
                return {f"{self.name}{i}": self.data[i] for i in range(self.dim)}
        else:
            return {self.name: self.data}

    def to_string(self):
        if not self.is_array:
            return str(self.data)
        else:
            return " ".join([str(data) for data in self.data])

    def __repr__(self):
        return f"{self.name}: {self.dtype}[{self.dim}]"


@dataclass
class SingleFloat(AttributeBase):
    dtype: Union[Type, str] = float
    is_array: bool = False

@dataclass
class SingleInt(AttributeBase):
    dtype: Union[Type, str] = int
    is_array: bool = False

@dataclass
class Position(AttributeBase):
    name: str = "pos"
    dtype: Union[Type, str] = float
    is_array: bool = True
    dim: int = 3
    urdf_path: tuple = ("origin", "xyz")

@dataclass
class Quaternion(AttributeBase):
    name: str = "quat"
    dtype: Union[Type, str] = float
    is_array: bool = True
    dim: int = 4
    urdf_path: tuple = ("origin", "rpy")

@dataclass
class Axis(AttributeBase):
    name: str = "axis"
    dtype: Union[Type, str] = float
    is_array: bool = True
    dim: int = 3
    urdf_path: tuple = ("axis", "xyz")

@dataclass
class Name(AttributeBase):
    name: str = "name"
    dtype: Union[Type, str] = str
    is_array: bool = False
    urdf_path: tuple = ("name",)

@dataclass
class BodyName(AttributeBase):
    name: str = "body_name"
    dtype: Union[Type, str] = str
    is_array: bool = False

@dataclass
class JointName(AttributeBase):
    name: str = "joint_name"
    dtype: Union[Type, str] = str
    is_array: bool = False

@dataclass
class Id(SingleInt):
    name: str = "id"
    dtype: Union[Type, str] = int
    is_array: bool = False
    urdf_path: tuple = ("name",)

@dataclass
class Inertia(AttributeBase):
    name: str = "inertia"
    dtype: Union[Type, str] = float
    is_array: bool = True
    dim: int = 3
    urdf_path: tuple = ("inertial", "inertia", ("ixx", "iyy", "izz"))

@dataclass
class Range(AttributeBase):
    name: str = "range"
    dtype: Union[Type, str] = float
    is_array: bool = True
    dim: int = 2
    urdf_path: tuple = ("limit", ("lower", "upper"))
