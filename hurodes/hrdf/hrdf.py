import json
from pathlib import Path

from hurodes.hrdf.base.info import InfoBase, load_csv, save_csv
from hurodes.hrdf.infos import INFO_DICT
from hurodes.utils.mesh import simplify_obj


class HumanoidRobot:
    def __init__(self):
        self.body_parent_id: list[int] = []
        self.ground_dict: dict[str, InfoBase] = {}
        self.mesh_file_type = None

        self.info_list = {
            "body": [],
            "joint": [],
            "actuator": [],
            "mesh": [],
            "simple_geom": [],
        }

    @classmethod
    def from_dir(cls, hrdf_path: str):
        instance = cls()
        with open(Path(hrdf_path, "meta.json"), "r") as f:
            meta_info = json.load(f)
        instance.body_parent_id = meta_info["body_parent_id"]
        instance.mesh_file_type = meta_info["mesh_file_type"]
        instance.ground_dict = meta_info["ground"]
    
        for name in ["body", "joint", "actuator", "mesh", "simple_geom"]:
            component_csv = Path(hrdf_path, f"{name}.csv")
            if component_csv.exists():
                instance.info_list[name] = load_csv(str(component_csv), INFO_DICT[name])
        return instance

    def find_info_by_attr(self, attr_name: str, attr_value: str, info_name: str, single=False):
        res = []
        for info in self.info_list[info_name]:
            if info[attr_name].data == attr_value:
                res.append(info)
        
        if single:
            assert len(res) == 1, f"Found multiple info with attr {attr_name} = {attr_value}"
            return res[0]
        else:
            return res

    def save(self, save_path, mesh_path=None, max_faces=8000):
        """
        Save the HumanoidRobot to HRDF format.
        
        Args:
            save_path: Directory path to save the HRDF files
            mesh_path: Dictionary mapping mesh names to source paths (optional, for mesh processing)
            max_faces: Maximum number of faces for mesh simplification
        """
        save_path = Path(save_path)
        save_path.mkdir(parents=True, exist_ok=True)
        
        # Process mesh files if mesh_path is provided
        if mesh_path:
            for name, path in mesh_path.items():
                new_mesh_file = save_path / "meshes" / f"{name}.{self.mesh_file_type}"
                new_mesh_file.parent.mkdir(parents=True, exist_ok=True)
                simplify_obj(path, new_mesh_file, max_faces)
        
        # Save metadata
        meta_path = save_path / "meta.json"
        meta_path.touch(exist_ok=True)
        meta_info = {
            "body_parent_id": self.body_parent_id,
            "mesh_file_type": self.mesh_file_type,
            "ground": self.ground_dict
        }
        with open(meta_path, "w") as json_file:
            json.dump(meta_info, json_file, indent=4)
        
        # Save component CSV files
        for info_name in ["body", "joint", "actuator", "mesh", "simple_geom"]:
            info_list = self.info_list[info_name]
            if len(info_list) > 0:
                save_csv(info_list, save_path / f"{info_name}.csv")
