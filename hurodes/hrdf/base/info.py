from typing import List, Type, Dict, Any, ClassVar, Optional, Union
import numpy as np
import pandas as pd
import re

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
        info_dict = cls.specific_parse_mujoco(info_dict, part_model, part_spec, whole_model, whole_spec)

        for attr_class in cls.attr_classes:
            if attr_class.mujoco_name is None:
                info_dict[attr_class.name] = None
            elif attr_class.name not in info_dict:
                info_dict[attr_class.name] = getattr(part_model, attr_class.mujoco_name)
        return cls.from_dict(info_dict)

    @classmethod
    def specific_parse_mujoco(cls, info_dict, part_model, part_spec=None, whole_model=None, whole_spec=None):
        return info_dict

    def to_mujoco_dict(self, tag=None):
        mujoco_dict = {}
        for attr_class in self.attr_classes:
            if attr_class.mujoco_name is not None:
                mujoco_dict[attr_class.mujoco_name] = self[attr_class.name].to_string()
        mujoco_dict = self.specific_generate_mujoco(mujoco_dict, tag)

        mujoco_dict = {k: v for k, v in mujoco_dict.items() if v != "nan"}
        # Ensure all values are strings to prevent XML serialization errors
        mujoco_dict = {k: str(v) if v is not None else "0" for k, v in mujoco_dict.items()}
        return mujoco_dict

    def specific_generate_mujoco(self, mujoco_dict, tag=None):
        return mujoco_dict

    def to_urdf_dict(self, tag=None):
        """
        Convert Info object to URDF dictionary format.
        
        Args:
            tag: URDF tag type to generate attributes for (e.g., 'link', 'joint', 'limit', etc.)
            
        Returns:
            Dictionary with URDF-specific attribute names and values
        """
        urdf_dict = {}
        for attr_class in self.attr_classes:
            # Use attribute name as URDF name by default (can be overridden by urdf_name)
            urdf_name = getattr(attr_class, 'urdf_name', attr_class.name)
            urdf_dict[urdf_name] = self[attr_class.name].to_string()
        urdf_dict = self.specific_generate_urdf(urdf_dict, tag)

        urdf_dict = {k: v for k, v in urdf_dict.items() if v != "nan"}
        # Ensure all values are strings to prevent XML serialization errors
        urdf_dict = {k: str(v) if v is not None else "0" for k, v in urdf_dict.items()}
        return urdf_dict

    def specific_generate_urdf(self, urdf_dict, tag=None):
        """
        Override this method in subclasses to provide URDF-specific transformations.
        
        Args:
            urdf_dict: Dictionary with raw attribute values
            tag: URDF tag type being generated
            
        Returns:
            Transformed dictionary for the specific URDF tag
        """
        return urdf_dict

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
