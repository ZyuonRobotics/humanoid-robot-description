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
        dim: Dimension of the attribute.
        mujoco_name: Name of the attribute in Mujoco, if None, do not generate mujoco attribute.
        urdf_path: Path to the attribute in URDF, if None, do not generate urdf attribute.
    """
    name: str = ""
    dtype: Union[Type, str] = float
    dim: int = 0
    mujoco_name: str = None
    urdf_path: tuple = None
    default_value: Any = None # only used for generating

    def __init_subclass__(cls):
        if isinstance(cls.dtype, str):
            assert cls.dtype in TYPE_MAP, f"Invalid data type: {cls.dtype}"
            cls.dtype = TYPE_MAP[cls.dtype]
        else:
            assert cls.dtype in TYPE_MAP.values(), f"Invalid data type: {cls.dtype}"
        
        assert cls.dim >= 0, "Dimension must be non-negative"
    
    def __post_init__(self):
        self._data = None

    @property
    def data(self) -> Any:
        return self._data

    @data.setter
    def data(self, data: Optional[Union[int, str, float, bool, np.ndarray, list]]):
        if data is None:
            pass
        elif self.dim == 0:
            data = self.dtype(data)
        else:
            if isinstance(data, list):
                data = np.array(data)
            
            assert isinstance(data, np.ndarray), f"Invalid data type: {type(data)}, expected: np.ndarray"
            assert data.shape == (self.dim,), f"Invalid data shape: {data.shape}, expected: {self.dim}"
            data = data.astype(self.dtype) if self.dtype != str else data
        self._data = data

    def parse_flat_dict(self, flat_dict: Dict[str, Any]):
        """
        Parse flat dictionary to attribute data. This function assumes that the value must be in the flat_dict.
        """
        assert flat_dict is not None and isinstance(flat_dict, dict), "flat_dict must be a dictionary"
        if self.dim == 0:
            assert self.name in flat_dict, f"Attribute {self.name} not found in flat_dict"
            self.data = flat_dict[self.name]
        else:
            for i in range(self.dim):
                assert f"{self.name}{i}" in flat_dict, f"Attribute {self.name}{i} not found in attr_dict"
            data = [flat_dict[f"{self.name}{i}"] for i in range(self.dim)]
            if all([data[i] is None for i in range(self.dim)]):
                data = None
            else:
                self.data = data

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
        if self.dim > 0:
            if self.data is None:
                return {f"{self.name}{i}": None for i in range(self.dim)}
            else:
                return {f"{self.name}{i}": self.data[i] for i in range(self.dim)}
        else:
            return {self.name: self.data}

    def to_string(self, using_default: bool = False):
        if using_default:
            assert self.data is not None or self.default_value is not None, f"Data is None for Attribute {self.name}"
            data = self.data if self.data is not None else self.default_value
        else:
            assert self.data is not None, f"Data is None for Attribute {self.name}"
            data = self.data

        if self.dim == 0:
            if self.dtype == str and str(data) == "nan":
                return ""
            else:
                return str(data)
        else:
            return " ".join([str(data) for data in data])

@dataclass
class Position(AttributeBase):
    name: str = "pos"
    dim: int = 3
    mujoco_name: str = "pos"
    urdf_path: tuple = ("origin", "xyz")

@dataclass
class Quaternion(AttributeBase):
    name: str = "quat"
    dim: int = 4
    mujoco_name: str = "quat"

@dataclass
class Axis(AttributeBase):
    name: str = "axis"
    dim: int = 3
    mujoco_name: str = "axis"
    urdf_path: tuple = ("axis", "xyz")

@dataclass
class Name(AttributeBase):
    name: str = "name"
    dtype: Union[Type, str] = str
    mujoco_name: str = "name"
    urdf_path: tuple = ("name",)

@dataclass
class BodyName(Name):
    # BodyName should be a subclass of Name, which will be addressed when generate mjcf
    name: str = "body_name"
    dtype: Union[Type, str] = str
    mujoco_name = None
    urdf_path = None

@dataclass
class JointName(Name):
    # JointName should be a subclass of Name, which will be addressed when generate mjcf
    name: str = "joint_name"
    dtype: Union[Type, str] = str
    mujoco_name = None
    urdf_path = None

@dataclass
class Id(AttributeBase):
    name: str = "id"
    dtype: Union[Type, str] = int
