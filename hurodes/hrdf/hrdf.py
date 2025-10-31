from pathlib import Path
from dataclasses import dataclass
from typing import Optional, Union, Any
from functools import partial

import yaml

from hurodes.hrdf.base.info import InfoBase, InfoList
from hurodes.hrdf.infos import INFO_CLASS_DICT
from hurodes.utils.mesh import simplify_obj
from hurodes import ROBOTS_PATH
from hurodes.utils.config import BaseConfig
from hurodes.joint_mapping.joint_mapping_config import JointMappingConfig


class GroundConfig(BaseConfig):
    type: str = None
    contact_affinity: int = None
    contact_type: int = None
    friction: float = None

class SimulatorConfig(BaseConfig):
    timestep: float = None
    gravity: list[float] = None
    ground: GroundConfig = None

class BodyNameConfig(BaseConfig):
    torso_name: str = None
    hip_names: list[str] = None
    knee_names: list[str] = None
    foot_names: list[str] = None

class DeviceConfig(BaseConfig):
    device_type: str = None
    device_config_name: str = None

    @property
    def name(self):
        if self.device_type is not None and self.device_config_name is not None:
            return f"{self.device_type}_{self.device_config_name}"
        else:
            return None

class IMUConfig(DeviceConfig):
    position: list[float] = None
    orientation: list[float] = None
    body_name: str = None
    value: list[str] = None

class MotorConfig(DeviceConfig):
    pass

class HRDF(BaseConfig):
    model_config = {"arbitrary_types_allowed": True}
    
    robot_name: str = None
    # from meta.yaml
    body_parent_id: list[int] = None
    mesh_file_type: str = None
    simulator_config: SimulatorConfig = None
    body_name_config: BodyNameConfig = None
    imu_config_list: list[IMUConfig] = None
    motor_config_list: list[MotorConfig] = None
    # from joint_mapping.yaml
    joint_mapping_config: JointMappingConfig = None
    # from component CSV files
    info_list_dict: dict[str, InfoList] = None

    @classmethod
    def from_dir(cls, hrdf_path: Union[Path, str]):
        if isinstance(hrdf_path, str):
            hrdf_path = Path(hrdf_path)
        assert hrdf_path.exists(), f"HRDF path not found: {hrdf_path}"
        assert hrdf_path.is_dir(), f"HRDF path is not a directory: {hrdf_path}"
        assert (hrdf_path / "meta.yaml").exists(), f"meta.yaml not found in HRDF path: {hrdf_path}"

        with open(hrdf_path / "meta.yaml", "r", encoding='utf-8') as f:
            meta_info = yaml.safe_load(f)
        instance = cls.from_dict(meta_info)
        instance.robot_name = hrdf_path.relative_to(ROBOTS_PATH).as_posix()

        if (hrdf_path / "joint_mapping.yaml").exists():
            instance.joint_mapping_config = JointMappingConfig.from_yaml(hrdf_path / "joint_mapping.yaml")

        for info_list_name, info_list_class in INFO_CLASS_DICT.items():
            component_csv = hrdf_path / f"{info_list_name}.csv"
            if component_csv.exists():
                if instance.info_list_dict is None:
                    instance.info_list_dict = {}
                instance.info_list_dict[info_list_name] = InfoList.from_csv(component_csv, info_list_class)
        return instance

    def save(self, mesh_path: Optional[dict[str, Union[Path, str]]] = None, max_faces: int = 8000):
        self.hrdf_path.mkdir(parents=True, exist_ok=True)
        
        # Process mesh files if mesh_path is provided
        if mesh_path:
            for name, path in mesh_path.items():
                new_mesh_file = self.hrdf_path / "meshes" / f"{name}.{self.mesh_file_type}"
                new_mesh_file.parent.mkdir(parents=True, exist_ok=True)
                if path != new_mesh_file:
                    simplify_obj(path, new_mesh_file, max_faces)
        
        # Save metadata
        meta_path = self.hrdf_path / "meta.yaml"
        meta_path.touch(exist_ok=True)
        self.to_yaml(meta_path)
        
        # Save component CSV files
        for info_name, info_list in self.info_list_dict.items():
            if len(info_list) > 0:
                info_list.save_csv(self.hrdf_path / f"{info_name}.csv")

        # Save joint mapping config
        if self.joint_mapping_config is not None:
            self.joint_mapping_config.to_yaml(self.hrdf_path / "joint_mapping.yaml")

    def fix_simple_geom(self):
        if "simple_geom" in self.info_list_dict:
            for idx, simple_geom in enumerate(self.info_list_dict["simple_geom"]):
                if simple_geom["name"].data == "":
                    simple_geom["name"].data = f"geom_{idx}"

    def add_info(self, info_name: str, info: InfoBase):
        assert info_name in INFO_CLASS_DICT, f"Info name {info_name} not found"
        if self.info_list_dict is None:
            self.info_list_dict = {}
        if info_name not in self.info_list_dict:
            self.info_list_dict[info_name] = InfoList(INFO_CLASS_DICT[info_name])
        self.info_list_dict[info_name].append(info)

    def get_info_by_attr(self, attr_name: str, attr_value: str, info_name: str, single=False):
        assert info_name in INFO_CLASS_DICT, f"Info name {info_name} not found"
        return self.info_list_dict[info_name].get_info_by_attr(attr_name, attr_value, single)
    
    def get_info_data_dict(self, info_name: str, key_attr: str, value_attr: str):
        assert info_name in INFO_CLASS_DICT, f"Info name {info_name} not found"
        return self.info_list_dict[info_name].get_data_dict(key_attr, value_attr)

    def get_info_data_list(self, info_name: str, attr: str):
        assert info_name in INFO_CLASS_DICT, f"Info name {info_name} not found"
        return self.info_list_dict[info_name].get_data_list(attr)

    @property
    def hrdf_path(self):
        return ROBOTS_PATH / self.robot_name
    
    # joint
    @property
    def joint_armature_dict(self):
        return self.get_info_data_dict("joint", "name", "armature")
    
    @property
    def joint_static_friction_dict(self):
        return self.get_info_data_dict("joint", "name", "static_friction")

    @property
    def joint_dynamic_friction_dict(self):
        return self.get_info_data_dict("joint", "name", "dynamic_friction")

    @property
    def joint_viscous_friction_dict(self):
        return self.get_info_data_dict("joint", "name", "viscous_friction")

    @property
    def joint_range_dict(self):
        return self.get_info_data_dict("joint", "name", "range")

    @property
    def joint_name_list(self):
        return self.get_info_data_list("joint", "name")

    # actuator
    @property
    def actuator_peak_velocity_dict(self):
        return self.get_info_data_dict("actuator", "joint_name", "peak_velocity")

    @property
    def actuator_peak_torque_dict(self):
        return self.get_info_data_dict("actuator", "joint_name", "peak_torque")

    @property
    def actuator_p_gain_dict(self):
        return self.get_info_data_dict("actuator", "joint_name", "p_gain")

    @property
    def actuator_d_gain_dict(self):
        return self.get_info_data_dict("actuator", "joint_name", "d_gain")
    
    @property
    def base_height(self):
        return float(self.info_list_dict["body"][0]["pos"].data[2])

    @property
    def hip_names(self):
        return self.body_name_config.hip_names

    @property
    def knee_names(self):
        return self.body_name_config.knee_names

    @property
    def foot_names(self):
        return self.body_name_config.foot_names

    @property
    def torso_name(self):
        return self.body_name_config.torso_name
    
    @property
    def imu_dict(self):
        return {imu_config.name: imu_config.value for imu_config in self.imu_config_list}

    @property
    def motor_dict(self):
        return {motor_config.name: motor_config.value for motor_config in self.motor_config_list}
    
    def to_dict(self)-> dict[str, Any]:
        res = {}
        for key, value in self.model_dump().items():
            if key in ["body_parent_id", "mesh_file_type", "simulator_config", "body_name_config", "imu_config_list", "motor_config_list"]:
                res[key] = value
        return res