from pathlib import Path
from dataclasses import dataclass
from typing import Optional
from functools import partial

import yaml

from hurodes.hrdf.base.info import InfoBase, load_csv, save_csv
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

class HRDF:
    def __init__(self, info_class_dict: Optional[dict[str, type[InfoBase]]] = None, **kwargs):
        self.info_class_dict = info_class_dict or INFO_CLASS_DICT
        self.info_list = {name: [] for name in self.info_class_dict.keys()}

        self.robot_name = None
        self.body_parent_id: list[int] = []
        self.mesh_file_type = None
        self.simulator_config = None

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
        instance.mesh_file_type = meta_info["mesh_file_type"]
        instance.simulator_config = SimulatorConfig.from_dict(meta_info["simulator_config"])
    
        for name in ["body", "joint", "actuator", "mesh", "simple_geom"]:
            component_csv = Path(hrdf_path, f"{name}.csv")
            if component_csv.exists():
                instance.info_list[name] = load_csv(str(component_csv), instance.info_class_dict[name])
        return instance

    @property
    def hrdf_path(self):
        return ROBOTS_PATH / self.robot_name
    
    def find_info_by_attr(self, attr_name: str, attr_value: str, info_name: str, single=False):
        res = []
        for info in self.info_list[info_name]:
            if info[attr_name].data == attr_value:
                res.append(info)
        
        if single:
            if len(res) == 0:
                raise ValueError(f"No info found with attr {attr_name} = {attr_value}")
            elif len(res) > 1:
                raise ValueError(f"Found multiple info with attr {attr_name} = {attr_value}")
            else:
                return res[0]
        else:
            return res

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
                simplify_obj(path, new_mesh_file, max_faces)
        
        # Save metadata
        meta_path = self.hrdf_path / "meta.yaml"
        meta_path.touch(exist_ok=True)
        meta_info = {
            "body_parent_id": self.body_parent_id,
            "mesh_file_type": self.mesh_file_type,
            "simulator_config": self.simulator_config.to_dict()
        }
        with open(meta_path, "w", encoding='utf-8') as yaml_file:
            yaml.dump(meta_info, yaml_file, default_flow_style=False, allow_unicode=True, indent=2)
        
        # Save component CSV files
        for info_name in ["body", "joint", "actuator", "mesh", "simple_geom"]:
            info_list = self.info_list[info_name]
            if len(info_list) > 0:
                save_csv(info_list, self.hrdf_path / f"{info_name}.csv")

    def fix_simple_geom(self):
        for idx, simple_geom in enumerate(self.info_list["simple_geom"]):
            if simple_geom["name"].data == "":
                simple_geom["name"].data = f"geom_{idx}"

    def get_info_data_dict(self, info_name: str, key_attr: str, value_attr: str):
        assert info_name in self.info_class_dict, f"Info name {info_name} not found"
        return {info[key_attr].data: info[value_attr].data for info in self.info_list[info_name]}

    def get_info_data_list(self, info_name: str, attr: str):
        assert info_name in self.info_class_dict, f"Info name {info_name} not found"
        return [info[attr].data for info in self.info_list[info_name]]

    @property
    def armature_dict(self):
        return self.get_info_data_dict("joint", "name", "armature")

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
        return float(self.info_list["body"][0]["pos"].data[2])