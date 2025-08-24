from typing import List, Type, Dict, Any, ClassVar, Optional, Union
import numpy as np
import pandas as pd
import re
from dataclasses import dataclass

from hurodes.hrdf.base.attribute import Attribute



class Info:
    type_map = {'int': int, 'float': float, 'str': str, 'bool': bool}

    def __init__(self, attr: Attribute):
        self.name = attr.name
        self.dtype = attr.dtype
        self.is_array = attr.is_array
        self.dim = attr.dim

        if isinstance(self.dtype, str):
            self.dtype = self.type_map[self.dtype]
        else:
            assert isinstance(self.dtype, Type), f"Invalid data type: {self.dtype}"

        if self.is_array:
            assert self.dim > 1, "Array must have dimension > 1"
        else:
            assert self.dim == 0

        self._data = None

    def type_convert(self, data: Any) -> Any:
        try:
            return self.dtype(data)
        except ValueError:
            raise ValueError(f"Invalid data type: {self.dtype}")
    
    def parse_string(self, string: str):
        if not self.is_array:
            data = self.type_convert(string)
        else:
            data = string.split()
            assert len(data) == self.dim, f"Expected {self.dim} elements, but got {len(data)}"
            data = [self.type_convert(elem) for elem in data]
            if self.dtype != str:
                data = np.array(data)

        self._data = data

    def parse_dict(self, attr_dict: Dict[str, Any]):
        assert attr_dict is not None and isinstance(attr_dict, dict), "attr_dict must be a dictionary"
        
        if not self.is_array:
            assert self.name in attr_dict, f"Attribute {self.name} not found in attr_dict"
            data = self.type_convert(attr_dict[self.name])
        else:
            for i in range(self.dim):
                assert f"{self.name}{i}" in attr_dict, f"Attribute {self.name}{i} not found in attr_dict"
            data = [self.type_convert(attr_dict[f"{self.name}{i}"]) for i in range(self.dim)]
            if self.dtype != str:
                data = np.array(data)

        self._data = data

    @property
    def data(self) -> Any:
        return self._data

    @data.setter
    def data(self, data: Union[int, str, float, bool, np.ndarray]):
        if not self.is_array:
            data = self.type_convert(data)
        else:
            assert isinstance(data, np.ndarray), f"Invalid data type: {type(data)}, expected: np.ndarray"
            assert data.shape == (self.dim,), f"Invalid data shape: {data.shape}, expected: {self.dim}"
            data = data.astype(self.dtype)
        self._data = data

    
    def to_dict(self) -> Dict[str, Any]:
        if not self.is_array:
            if self.data is None:
                return {self.name: None}
            else:
                return {self.name: self.dtype(self.data)}
        else:
            if self.data is None:
                return {f"{self.name}{i}": None for i in range(self.dim)}
            else:
                return {f"{self.name}{i}": self.dtype(self.data[i]) for i in range(self.dim)}

class Infos:
    def __init__(self, attrs: List[Attribute] = None):
        self.attrs = attrs
        self.attr_names = [attr.name for attr in attrs]

        assert len(self.attr_names) == len(set(self.attr_names)), "Duplicate attribute names found in attrs"

        self._infos = None
    
    def parse_flat_dict(self, info_dict: Dict[str, Any]):
        assert info_dict is not None, "info_dict is required"
        assert isinstance(info_dict, dict), "info_dict must be a dictionary"
        
        assert len(info_dict) == len(set(info_dict.keys())), "Duplicate attribute names found in info_dict"
        assert set(info_dict.keys()) == set(self.attr_names), "Attribute names in info_dict do not match ATTR_CLASSES"

        # for attr_name, attr_data in info_dict.items():
        #     self.infos[attr_name].data = attr_data # type: ignore
        pass

    def to_flat_dict(self) -> Dict[str, Any]:
        """
        Convert BaseAttrList to dictionary format
        
        Returns:
            Dict[str, Any]: Dictionary containing all attributes in key-value format
        """
        if self._infos is None:
            return {}
        
        result = {}
        for name, info in self._infos.items():
            result.update(info.to_dict())
        
        return result

    def parse_dict(self, info_dict: Dict[str, Any]):
        self._infos = {}
        for attr in self.attrs:
            self._infos[attr.name] = Info(attr)
            if info_dict[attr.name] is not None:
                self._infos[attr.name].data = info_dict[attr.name]

    @classmethod
    def from_mujoco(cls, part_model, part_spec=None, whole_model=None, whole_spec=None):
        """
        This function is used to parse the information from the mujoco model and spec.
        """
        infos = cls()

        info_dict = {}
        info_dict = infos.specific_parse_mujoco(info_dict, part_model, part_spec, whole_model, whole_spec)
        
        for attr in infos.attrs:
            if attr.name not in info_dict:
                info_dict[attr.name] = getattr(part_model, attr.mujoco_name)
        infos.parse_dict(info_dict)
        return infos

    def specific_parse_mujoco(self, info_dict, part_model, part_spec=None, whole_model=None, whole_spec=None):
        return info_dict

def save_csv(infos_list: List[Infos], save_path: str):
    assert len(infos_list) > 0, "infos_list is empty"
    
    df_list = [infos.to_flat_dict() for infos in infos_list]
        
    df = pd.DataFrame(df_list)
    df.to_csv(save_path, index=False)