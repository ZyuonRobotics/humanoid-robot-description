from pathlib import Path
from typing import Dict, Any, Union
from abc import ABC

from pydantic import BaseModel
import yaml


class BaseConfig(BaseModel, ABC):    
    @classmethod
    def from_dict(cls, config_dict: Dict[str, Any]) -> 'BaseConfig':
        return cls(**config_dict)
    
    @classmethod
    def from_yaml(cls, yaml_path: Union[str, Path]) -> 'BaseConfig':
        with open(yaml_path, 'r', encoding='utf-8') as f:
            config_dict = yaml.safe_load(f)
        return cls.from_dict(config_dict)
    
    def to_yaml(self, yaml_path: Union[str, Path]) -> None:
        with open(yaml_path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(self.to_dict(), f, default_flow_style=False, allow_unicode=True)
    
    def to_dict(self) -> Dict[str, Any]:
        return self.model_dump() 

    @property
    def has_none(self):
        # recursively check if any attribute is None
        for attr, value in self.model_dump().items():
            if value is None:
                return True
            if isinstance(value, BaseConfig):
                if value.has_none:
                    return True
        return False