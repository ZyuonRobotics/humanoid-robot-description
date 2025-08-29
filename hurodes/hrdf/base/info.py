from typing import List, Type, Dict, Any, ClassVar, Optional, Union
import numpy as np
import pandas as pd
import re
import xml.etree.ElementTree as ET

from bidict import bidict

from hurodes.hrdf.base.attribute import AttributeBase

class InfoBase:
    info_name: str = ""
    attr_classes: tuple[AttributeBase] = ()

    def parse_flat_dict(self, flat_dict: Dict[str, Any]):
        assert flat_dict is not None, "info_dict is required"
        assert isinstance(flat_dict, dict), "info_dict must be a dictionary"
        
        self._dict: Dict[str, AttributeBase] = {}
        for attr_class in self.attr_classes:
            self._dict[attr_class.name] = attr_class.from_flat_dict(flat_dict)

    @classmethod
    def from_flat_dict(cls, flat_dict: Dict[str, Any]):
        info = cls()
        info.parse_flat_dict(flat_dict)
        return info

    def to_flat_dict(self) -> Dict[str, Any]:
        result = {}
        for attr_value in self._dict.values():
            result.update(attr_value.to_dict())
        return result

    def parse_dict(self, info_dict: Dict[str, Any]):
        self._dict = {}
        for attr_class in self.attr_classes:
            if attr_class.name not in info_dict:
                info_dict[attr_class.name] = None
            self._dict[attr_class.name] = attr_class.from_data(info_dict[attr_class.name])

    @classmethod
    def from_dict(cls, info_dict: Dict[str, Any]):
        info = cls()
        info.parse_dict(info_dict)
        return info

    @classmethod
    def from_mujoco(cls, part_model, part_spec=None, whole_model=None, whole_spec=None):
        """
        This function is used to parse the information from the mujoco model and spec.
        """
        info_dict = {}
        for attr_class in cls.attr_classes:
            if attr_class.mujoco_name is not None:
                info_dict[attr_class.name] = getattr(part_model, attr_class.mujoco_name)
        info_dict = cls._specific_parse_mujoco(info_dict, part_model, part_spec, whole_model, whole_spec)
        return cls.from_dict(info_dict)

    @classmethod
    def _specific_parse_mujoco(cls, info_dict, part_model, part_spec=None, whole_model=None, whole_spec=None):
        return info_dict

    def to_mujoco_dict(self, tag=None):
        mujoco_dict, extra_dict = {}, {}
        for attr_class in self.attr_classes:
            extra_dict[attr_class.name] = self[attr_class.name].to_string()
            if attr_class.mujoco_name is not None:
                mujoco_dict[attr_class.mujoco_name] = self[attr_class.name].to_string()                
        mujoco_dict = self._specific_generate_mujoco(mujoco_dict, extra_dict, tag)
        return mujoco_dict

    def _specific_generate_mujoco(self, mujoco_dict, extra_dict, tag=None):
        return mujoco_dict

    def _to_urdf_dict(self, tag=None):
        urdf_dict, extra_dict = {}, {}
        for attr_class in self.attr_classes:
            extra_dict[attr_class.name] = self[attr_class.name].to_string()
            if attr_class.urdf_path is not None:
                urdf_dict[attr_class.urdf_path] = self[attr_class.name].to_string()                
        urdf_dict = self._specific_generate_urdf(urdf_dict, extra_dict, tag)
        return urdf_dict, extra_dict

    def _specific_generate_urdf(self, urdf_dict, extra_dict, tag=None):
        return urdf_dict

    def to_urdf_elem(self, root_elem, tag=None):
        urdf_dict, extra_dict = self._to_urdf_dict(tag)
        for attr_path, attr_value in urdf_dict.items():
            add_attr_to_elem(root_elem, attr_path, attr_value)

    def __repr__(self):
        string = f"{self.info_name}(\n"
        for attr_class in self.attr_classes:
            string += f"    {attr_class}\n"
        string += ")"
        return string

    def __getitem__(self, key: str):
        assert key in self._dict, f"Attribute {key} not found in info"
        return self._dict[key]

def save_csv(info_list: List[InfoBase], save_path: str):
    assert len(info_list) > 0, "info_list is empty"
    df_list = [info.to_flat_dict() for info in info_list]
        
    df = pd.DataFrame(df_list)
    df.to_csv(save_path, index=False)


def load_csv(csv_path: str, info_class: type) -> List[InfoBase]:
    df = pd.read_csv(csv_path)
    df_list = df.to_dict('records')
    
    return [info_class.from_flat_dict(data_dict) for data_dict in df_list]

def find_info_by_attr(attr_name, attr_value, info_list, return_one=False):
    res = []
    for info in info_list:
        if info[attr_name].data == attr_value:
            res.append(info)
    if return_one:
        assert len(res) == 1, f"Found multiple info with attr {attr_name} = {attr_value}"
        return res[0]
    else:
        return res

def add_attr_to_elem(elem, attr_path, attr_value):
    if len(attr_path) == 1:
        if isinstance(attr_path[0], str):
            elem.set(attr_path[0], str(attr_value))
        elif isinstance(attr_path[0], tuple):
            values = attr_value.split()
            assert len(values) == len(attr_path[0]), f"Expected {len(attr_path[0])} values for {attr_path[0]}, got {len(values)}"
            for i, value in enumerate(values):
                elem.set(attr_path[0][i], value)
    else:
        sub_elem_list = elem.findall(attr_path[0])
        if len(sub_elem_list) == 0:
            sub_elem = ET.SubElement(elem, attr_path[0])
        elif len(sub_elem_list) == 1:
            sub_elem = sub_elem_list[0]
        else:
            raise ValueError(f"Found multiple elements with tag {attr_path[0]}")
        add_attr_to_elem(sub_elem, attr_path[1:], attr_value)
