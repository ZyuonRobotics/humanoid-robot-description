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

        self.data = None

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

        self.data = data

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

        self.data = data
    
    def to_dict(self) -> Dict[str, Any]:
        if not self.is_array:
            return {self.name: self.data}
        else:
            return {f"{self.name}{i}": self.data[i] for i in range(self.dim)}

class InfoList:
    def __init__(self, attrs: List[Attribute] = None):
        self.attrs = attrs

        self.infos = {attr.name: Info(attr) for attr in attrs}
    
    # def from_string_dict(cls, attr_dict: Dict[str, Any]) -> 'BaseAttrList':
    #     """Create BaseAttrList from dictionary"""
    #     assert attr_dict is not None, "attr_dict is required"
    #     assert isinstance(attr_dict, dict), "attr_dict must be a dictionary"
        
    #     assert len(attr_dict) == len(set(attr_dict.keys())), "Duplicate attribute names found in attr_dict"
    #     assert set(attr_dict.keys()) == set(cls.get_attr_names()), "Attribute names in attr_dict do not match ATTR_CLASSES"

    #     attrs = {}
    #     for attr_class in cls.ATTR_CLASSES:
    #         attr_name = attr_class.get_attr_name()
    #         attrs[attr_name] = attr_class.from_string(attr_dict[attr_name])

    #     instance = cls(attrs=attrs)
    #     return instance

    # def to_dict(self) -> Dict[str, Any]:
    #     """
    #     Convert BaseAttrList to dictionary format
        
    #     Returns:
    #         Dict[str, Any]: Dictionary containing all attributes in key-value format
    #     """
    #     if self.attrs is None:
    #         return {}
        
    #     result = {}
    #     for attr_instance in self.attrs.values():
    #         result.update(attr_instance.to_dict())
        
    #     return result
