from copy import deepcopy
from typing import List, Type, Dict, Any, ClassVar, Optional, Union
from pathlib import Path
import numpy as np
import pandas as pd
import re
import xml.etree.ElementTree as ET

from bidict import bidict

from hurodes.hrdf.base.attribute import AttributeBase, Name
from hurodes.utils.string import get_prefix_name
from hurodes.utils.xml import add_attr_to_elem, extract_attr_from_elem

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
    def from_mujoco(cls, part_model, part_spec=None, *args, **kwargs):
        """Parse information from mujoco model and spec"""
        info_dict = {}
        for attr_class in cls.attr_classes:
            if attr_class.mujoco_name is not None:
                info_dict[attr_class.name] = getattr(part_model, attr_class.mujoco_name)
        info_dict = cls._specific_parse_mujoco(info_dict, part_model, part_spec, *args, **kwargs)
        return cls.from_dict(info_dict)

    @classmethod
    def from_urdf(cls, elem, root_elem, **kwargs):
        """Parse information from URDF element
        
        Args:
            elem: URDF XML element (link or joint)
            urdf_path: Path to the URDF file for resolving relative paths
        """
        assert elem is not None, "URDF element is required"
        
        info_dict = {}
        for attr_class in cls.attr_classes:
            if attr_class.urdf_path is not None:
                info_dict[attr_class.name] = extract_attr_from_elem(elem, attr_class.urdf_path)
        
        info_dict = cls._specific_parse_urdf(info_dict, elem, root_elem, **kwargs)
        return cls.from_dict(info_dict)

    @classmethod
    def _specific_parse_mujoco(cls, info_dict, part_model, part_spec=None, **kwargs):
        return info_dict

    @classmethod
    def _specific_parse_urdf(cls, info_dict, elem, root_elem, **kwargs):
        return info_dict

    def to_mujoco_dict(self, tag=None, prefix=None):
        mujoco_dict, extra_dict = {}, {}
        for attr_class in self.attr_classes:
            # using copy here to avoid modifying the original attribute value
            attr_value = deepcopy(self[attr_class.name])
            if issubclass(attr_class, Name):
                attr_value.data = get_prefix_name(prefix, attr_value.to_string())
            extra_dict[attr_class.name] = attr_value

            if attr_class.mujoco_name is not None:
                mujoco_dict[attr_class.mujoco_name] = attr_value.to_string(using_default=True)
        
        mujoco_dict = self._specific_generate_mujoco(mujoco_dict, extra_dict, tag)
        return mujoco_dict

    def _specific_generate_mujoco(self, mujoco_dict, extra_dict: Dict[str, AttributeBase], tag=None):
        return mujoco_dict

    def _to_urdf_dict(self, tag=None):
        urdf_dict, extra_dict = {}, {}
        for attr_class in self.attr_classes:
            attr_value = deepcopy(self[attr_class.name])
            extra_dict[attr_class.name] = attr_value
            if attr_class.urdf_path is not None:
                urdf_dict[attr_class.urdf_path] = attr_value.to_string(using_default=True)                
        urdf_dict = self._specific_generate_urdf(urdf_dict, extra_dict, tag)
        return urdf_dict, extra_dict

    def _specific_generate_urdf(self, urdf_dict, extra_dict: Dict[str, AttributeBase], tag=None):
        return urdf_dict

    def to_urdf_elem(self, root_elem, tag=None):
        urdf_dict, extra_dict = self._to_urdf_dict(tag)
        for attr_path, attr_value in urdf_dict.items():
            add_attr_to_elem(root_elem, attr_path, attr_value)

    def __repr__(self):
        string = f"{self.info_name}:\n"
        for attr_value in self._dict.values():
            string += f"  {attr_value}\n"

        return string

    def __getitem__(self, key: str):
        assert key in self._dict, f"Attribute {key} not found in info {self.info_name}"
        return self._dict[key]

class InfoList:
    def __init__(self, info_class: Type[InfoBase], infos: List[InfoBase] = None):
        self.info_class = info_class
        self.infos = infos or []

    def append(self, info: InfoBase):
        self.infos.append(info)

    def get_data_list(self, attr_name: str):
        return [info[attr_name].data for info in self.infos]

    def get_data_dict(self, key_attr: str, value_attr: str):
        return {info[key_attr].data: info[value_attr].data for info in self.infos}

    def get_info_by_attr(self, attr_name: str, attr_value: str, single=False):
        res = []
        for info in self.infos:
            if info[attr_name].data == attr_value:
                res.append(info)
        
        if single:
            if len(res) == 0:
                raise ValueError(f"No info found with attr satisfies: {attr_name} == {attr_value} in info list {self.info_class.info_name}")
            elif len(res) > 1:
                raise ValueError(f"Found multiple info with attr satisfies: {attr_name} == {attr_value} in info list {self.info_class.info_name}")
            else:
                return res[0]
        else:
            return res

    def __len__(self):
        return len(self.infos)

    def __iter__(self):
        return iter(self.infos)

    def __getitem__(self, index: int):
        return self.infos[index]

    def save_csv(self, save_path: Union[Path, str]):
        assert len(self) > 0, "info_list is empty"
        df_list = [info.to_flat_dict() for info in self.infos]
            
        df = pd.DataFrame(df_list)
        df.to_csv(str(save_path), index=False)

    @classmethod
    def from_csv(cls, csv_path: Union[Path, str], info_class: Type[InfoBase]):
        df = pd.read_csv(str(csv_path))
        df = df.replace({np.nan: None})

        df_list = df.to_dict('records')
        infos = [info_class.from_flat_dict(data_dict) for data_dict in df_list]
        
        return cls(info_class, infos=infos)
