import json
from collections import defaultdict
from pathlib import Path
from abc import ABC

import pandas as pd

from hurodes.utils.spec_parsing import parse_mujoco_spec, get_mesh_dict
from hurodes.utils.mesh import simplify_obj

class BaseParser(ABC):
    def __init__(self, file_path):
        self.file_path = file_path
        self.model_dict = defaultdict(list)
        self.mesh_path = {}
        self.mesh_file_type = {}
        self.body_name2idx = {}
        self.ground_dict = {}
        self.body_parent_id = []

    @property
    def mujoco_spec(self):
        raise NotImplementedError("Subclasses must implement this method")

    def parse(self, base_link_name="base_link"):
        self.model_dict, self.ground_dict, self.body_parent_id = parse_mujoco_spec(self.mujoco_spec)
        self.mesh_path, self.mesh_file_type = get_mesh_dict(self.mujoco_spec, self.file_path)

    def save(self, save_path, max_faces=8000):
        save_path = Path(save_path)
        save_path.mkdir(parents=True, exist_ok=True)
        for name, path in self.mesh_path.items():
            new_mesh_file = save_path / "meshes" / f"{name}.{self.mesh_file_type[name]}"
            new_mesh_file.parent.mkdir(parents=True, exist_ok=True)
            simplify_obj(path, new_mesh_file, max_faces)
        meta_path = save_path / "meta.json"
        meta_path.touch(exist_ok=True)
        meta_info = {
            "body_parent_id": self.body_parent_id,
            "mesh_file_type": self.mesh_file_type,
            "ground": self.ground_dict
        }
        with open(meta_path, "w") as json_file:
            json.dump(meta_info, json_file, indent=4)
        for name, data_dict in self.model_dict.items():
            df = pd.DataFrame(data_dict).sort_index(axis=1)
            df.to_csv(save_path / f"{name}.csv", index=False)

    def print_body_tree(self, colorful=False):
        pass
