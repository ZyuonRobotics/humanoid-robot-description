import json
from pathlib import Path

from hurodes.hrdf.base.info import InfoBase, load_csv
from hurodes.hrdf.infos import INFO_DICT


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
