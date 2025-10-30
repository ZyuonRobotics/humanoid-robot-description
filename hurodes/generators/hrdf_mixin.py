from pathlib import Path
from abc import ABC
from typing import Optional, TYPE_CHECKING

from hurodes.hrdf.hrdf import HRDF
from hurodes.generators.generator_base import GeneratorBase
from hurodes.hrdf.hrdf import SimulatorConfig, IMUConfig
from hurodes import ROBOTS_PATH
from hurodes.hrdf.base.info import InfoList

class HRDFMixin(ABC):
    """
    Mixin class for HRDF robot generators.
    
    This mixin provides common functionality for both MJCF and URDF humanoid generators,
    including HRDF loading, path management, and validation of generator inheritance.
    
    Classes using this mixin must inherit from GeneratorBase or its subclasses.
    """
    
    def __init__(self, **kwargs):
        """
        Initialize the HRDF mixin.
        
        Args:
            hrdf_path: Path to the HRDF directory containing robot description
            **kwargs: Additional arguments passed to parent class
        """
        # Validate that the class inherits from GeneratorBase
        if not isinstance(self, GeneratorBase):
            raise TypeError(
                f"{self.__class__.__name__} must inherit from GeneratorBase or its subclasses "
                f"to use HRDFMixin. Current MRO: {[cls.__name__ for cls in self.__class__.__mro__]}"
            )
        
        # Call parent constructor with remaining kwargs
        super().__init__(**kwargs)
        
        self.hrdf: Optional[HRDF] = None

    def _load(self, **kwargs):
        assert "hrdf" in kwargs
        assert isinstance(kwargs["hrdf"], HRDF), "hrdf must be an instance of HRDF"

        self.hrdf = kwargs["hrdf"]
        # Check if this is an MJCF generator to set simulator config, using "self.__class__.__mro__" to avoid circular import
        if 'MJCFGeneratorBase' in [cls.__name__ for cls in self.__class__.__mro__]:
            self.simulator_config = self.hrdf.simulator_config

    def _clean(self):
        self.hrdf = None

    def _destroy(self):
        pass

    @classmethod
    def from_hrdf(cls, hrdf: HRDF, **kwargs):
        instance = cls(**kwargs)
        instance.load(hrdf=hrdf)
        return instance
    
    @classmethod
    def from_hrdf_path(cls, hrdf_path: Path, **kwargs):
        return cls.from_hrdf(HRDF.from_dir(hrdf_path=hrdf_path), **kwargs)

    @classmethod
    def from_robot_name(cls, robot_name: str, **kwargs):
        return cls.from_hrdf(HRDF.from_dir(ROBOTS_PATH / robot_name), **kwargs)

    @property
    def mesh_directory(self) -> Path:
        return self.hrdf.hrdf_path / "meshes"
    
    @property 
    def robot_name(self) -> str:
        return self.hrdf.robot_name
    
    def get_info_by_attr(self, attr_name: str, attr_value: str, info_type: str, single: bool = False):
        assert self.hrdf is not None, "HRDF not loaded"
        return self.hrdf.get_info_by_attr(attr_name, attr_value, info_type, single=single)
    
    @property
    def body_parent_id(self) -> list:
        assert self.hrdf is not None, "HRDF not loaded"
        return self.hrdf.body_parent_id
    
    def info_list(self, info_type: str) -> InfoList:
        assert self.hrdf is not None, "HRDF not loaded"
        return self.hrdf.info_list_dict[info_type]
    
    @property
    def mesh_file_type(self) -> str:
        assert self.hrdf is not None, "HRDF not loaded"
        return self.hrdf.mesh_file_type
    
    def validate_mesh_exists(self, mesh_name: str) -> Path:
        assert self.hrdf is not None, "HRDF not loaded"
        mesh_file = self.mesh_directory / f"{mesh_name}.{self.mesh_file_type}"
        if not mesh_file.exists():
            raise FileNotFoundError(f"Mesh file {mesh_file} does not exist")
        return mesh_file 
    
    @property
    def imu_configs(self) -> list[IMUConfig]:
        assert self.hrdf is not None, "HRDF not loaded"
        return self.hrdf.imu_config_list
