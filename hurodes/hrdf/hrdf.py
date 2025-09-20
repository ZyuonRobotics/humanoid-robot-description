from pathlib import Path
from dataclasses import dataclass
from typing import Optional
from functools import partial

import yaml

from hurodes.hrdf.base.info import InfoBase, InfoList
from hurodes.hrdf.infos import INFO_CLASS_DICT
from hurodes.utils.mesh import simplify_obj
from hurodes import ROBOTS_PATH
from hurodes.utils.config import BaseConfig


class GroundConfig(BaseConfig):
    type: str = "plane"
    contact_affinity: int = 1
    contact_type: int = 1
    friction: float = 1.0

class SimulatorConfig(BaseConfig):
    timestep: float = 0.002
    gravity: list[float] = [0, 0, -9.81]
    ground: GroundConfig = GroundConfig()

class BodyNameConfig(BaseConfig):
    torso_name: str = ""
    hip_names: list[str] = []
    knee_names: list[str] = []
    foot_names: list[str] = []

class IMUConfig(BaseConfig):
    position: list[float] = [0, 0, 0]
    orientation: list[float] = [1, 0, 0, 0]
    name: str = ""
    body_name: str = ""
    value: list[str] = []

class HRDF:
    def __init__(self, info_class_dict: Optional[dict[str, type[InfoBase]]] = None, **kwargs):
        self.info_class_dict = info_class_dict or INFO_CLASS_DICT
        self.info_list_dict = {name: InfoList(info_class) for name, info_class in self.info_class_dict.items()}

        self.robot_name = None
        self.body_parent_id: list[int] = []
        self.mesh_file_type = None
        self.simulator_config = None
        self.body_name_config = BodyNameConfig()
        self.imu_configs = []
    
    @classmethod
    def from_dir(cls, hrdf_path: Path):
        assert hrdf_path.exists(), f"HRDF path not found: {hrdf_path}"
        assert hrdf_path.is_dir(), f"HRDF path is not a directory: {hrdf_path}"
        assert (hrdf_path / "meta.yaml").exists(), f"meta.yaml not found in HRDF path: {hrdf_path}"

        instance = cls()
        with open(Path(hrdf_path, "meta.yaml"), "r", encoding='utf-8') as f:
            meta_info = yaml.safe_load(f)
        instance.robot_name = hrdf_path.relative_to(ROBOTS_PATH).as_posix()
        instance.body_parent_id = meta_info["body_parent_id"]
        instance.mesh_file_type = meta_info.get("mesh_file_type", None)
        instance.simulator_config = SimulatorConfig.from_dict(meta_info.get("simulator_config", {}))
        instance.body_name_config = BodyNameConfig.from_dict(meta_info.get("body_name_config", {}))
        instance.imu_configs = [IMUConfig.from_dict(imu_config) for imu_config in meta_info.get("imu_configs", [])]
        for name in ["body", "joint", "actuator", "mesh", "simple_geom"]:
            component_csv = Path(hrdf_path, f"{name}.csv")
            if component_csv.exists():
                instance.info_list_dict[name] = InfoList.from_csv(str(component_csv), instance.info_class_dict[name])
        return instance

    @property
    def hrdf_path(self):
        return ROBOTS_PATH / self.robot_name
    
    def find_info_by_attr(self, attr_name: str, attr_value: str, info_name: str, single=False):
        assert info_name in self.info_class_dict, f"Info name {info_name} not found"
        return self.info_list_dict[info_name].find_info_by_attr(attr_name, attr_value, single)

    def save(self, mesh_path=None, max_faces=8000):
        """
        Save the HumanoidRobot to HRDF format.
        
        Args:
            mesh_path: Dictionary mapping mesh names to source paths (optional, for mesh processing)
            max_faces: Maximum number of faces for mesh simplification
        """
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
        meta_info = {
            "body_parent_id": self.body_parent_id,
            "mesh_file_type": self.mesh_file_type,
            "simulator_config": self.simulator_config.to_dict(),
            "body_name_config": self.body_name_config.to_dict(),
            "imu_configs": [imu_config.to_dict() for imu_config in self.imu_configs]
        }
        with open(meta_path, "w", encoding='utf-8') as yaml_file:
            yaml.dump(meta_info, yaml_file, default_flow_style=False, allow_unicode=True, indent=2)
        
        # Save component CSV files
        for info_name in ["body", "joint", "actuator", "mesh", "simple_geom"]:
            if len(self.info_list_dict[info_name]) > 0:
                self.info_list_dict[info_name].save_csv(self.hrdf_path / f"{info_name}.csv")

    def fix_simple_geom(self):
        for idx, simple_geom in enumerate(self.info_list_dict["simple_geom"]):
            if simple_geom["name"].data == "":
                simple_geom["name"].data = f"geom_{idx}"

    def get_info_data_dict(self, info_name: str, key_attr: str, value_attr: str):
        assert info_name in self.info_class_dict, f"Info name {info_name} not found"
        return self.info_list_dict[info_name].get_data_dict(key_attr, value_attr)

    def get_info_data_list(self, info_name: str, attr: str):
        assert info_name in self.info_class_dict, f"Info name {info_name} not found"
        return self.info_list_dict[info_name].get_data_list(attr)

    @property
    def armature_dict(self):
        return self.get_info_data_dict("joint", "name", "armature")
    
    @property
    def static_friction_dict(self):
        return self.get_info_data_dict("joint", "name", "static_friction")

    @property
    def dynamic_friction_dict(self):
        return self.get_info_data_dict("joint", "name", "dynamic_friction")

    @property
    def viscous_friction_dict(self):
        return self.get_info_data_dict("joint", "name", "viscous_friction")

    @property
    def peak_velocity_dict(self):
        return self.get_info_data_dict("actuator", "joint_name", "peak_velocity")

    @property
    def peak_torque_dict(self):
        return self.get_info_data_dict("actuator", "joint_name", "peak_torque")

    @property
    def p_gain_dict(self):
        return self.get_info_data_dict("actuator", "joint_name", "p_gain")

    @property
    def d_gain_dict(self):
        return self.get_info_data_dict("actuator", "joint_name", "d_gain")

    @property
    def joint_names(self):
        return self.get_info_data_list("joint", "name")

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
        return {imu_config.name: imu_config.value for imu_config in self.imu_configs}
