# import os
from pathlib import Path
import xml.etree.ElementTree as ET
import json
import math
from collections import defaultdict
from copy import deepcopy

from colorama import Fore, Style
import numpy as np
import pandas as pd

from hurodes.mjcf_generator.generator_base import MJCFGeneratorBase
from hurodes.utils.typing import dict2str

def find_by_body_id(all_data, body_id):
    """
    Find all data with the same bodyid.
    """
    res = []
    for data in all_data:
        if data["bodyid"] == body_id:
            data_copy = deepcopy(data)
            del data_copy["bodyid"]
            res.append(data_copy)
    return res

def get_prefix_name(prefix, name):
    return f"{prefix}_{name}" if prefix else name

class UnifiedMJCFGenerator(MJCFGeneratorBase):
    def __init__(
            self,
            hrdf_path,
            disable_gravity=False,
            timestep=0.001
    ):
        super().__init__(disable_gravity=disable_gravity, timestep=timestep)
        self.hrdf_path = hrdf_path

        self.body_parent_id: list[int] = []
        self.data_dict: dict[str, list[dict]] = {}
        self.mesh_file_type: dict[str, str] = {}

        self.all_collision_names = []

    def load(self):
        with open(Path(self.hrdf_path, "meta.json"), "r") as f:
            meta_info = json.load(f)
        self.body_parent_id = meta_info["body_parent_id"]
        self.mesh_file_type = meta_info["mesh_file_type"]
        self.ground_dict = meta_info["ground"]

        self.data_dict = {}
        for name in ["body", "joint", "mesh", "collision", "actuator"]:
            component_csv = Path(self.hrdf_path, f"{name}.csv")
            if component_csv.exists():
                self.data_dict[name] = pd.read_csv(component_csv).to_dict("records")

    def generate_single_body_xml(self, parent_node, body_idx, prefix=None):
        # body element
        body_data = self.data_dict["body"][body_idx]
        body_elem = ET.SubElement(parent_node, 'body')
        body_elem.set("name", get_prefix_name(prefix, dict2str(body_data, "name")))
        for key in ["pos", "quat"]:
            body_elem.set(key, dict2str(body_data, key))

        # inertial element
        inertial_elem = ET.SubElement(body_elem, 'inertial')
        for key_xml, key_mj in zip(["mass", "pos", "quat", "diaginertia"], ["mass", "ipos", "iquat", "inertia"]):
            inertial_elem.set(key_xml, dict2str(body_data, key_mj))

        # joint element
        joint_data_list = find_by_body_id(self.data_dict["joint"], body_idx)
        assert len(joint_data_list) == 1
        joint_data = joint_data_list[0]
        joint_elem = ET.SubElement(body_elem, 'joint')
        joint_elem.set("name", get_prefix_name(prefix, dict2str(joint_data, "name")))
        for key in ["type", "pos", "axis", "range", "damping", "stiffness", "armature", "frictionloss"]:
            joint_elem.set(key, dict2str(joint_data, key))

        # mesh element
        if "mesh" in self.data_dict:
            mesh_data_list = find_by_body_id(self.data_dict["mesh"], body_idx)
            for mesh_data in mesh_data_list:
                mesh_elem = ET.SubElement(body_elem, 'geom')
                mesh_elem.set("mesh", get_prefix_name(prefix, mesh_data["mesh"]))
                self.all_collision_names.append(mesh_data["mesh"])
                for key in ["type", "contype", "conaffinity", "pos", "quat", "rgba"]:
                    mesh_elem.set(key, dict2str(mesh_data, key))

        # collision element
        if "collision" in self.data_dict:
            collision_data_list = find_by_body_id(self.data_dict["collision"], body_idx)
            for idx, collision_data in enumerate(collision_data_list):
                collision_name = f"{dict2str(body_data, 'name')}_{idx}_{collision_data['type']}"
                collision_elem = ET.SubElement(body_elem, 'geom')
                self.all_collision_names.append(collision_name)
                collision_elem.set("rgba", "0 0.7 0.3 0.1")
                for key in ["type", "pos", "quat", "size", "contype", "conaffinity", "friction"]:
                    collision_elem.set(key, dict2str(collision_data, key))

        return body_elem

    def add_all_body(self, parent=None, current_index=0, prefix=None):
        if parent is None:
            parent = self.get_elem("worldbody")
        for child_index, parent_idx in enumerate(self.body_parent_id):
            if child_index == parent_idx: # skip world body
                continue
            elif parent_idx == current_index:
                body_elem = self.generate_single_body_xml(parent, child_index, prefix=prefix)
                self.add_all_body(body_elem, child_index, prefix=prefix)

    def add_compiler(self):
        self.get_elem("compiler").attrib = {
            "angle": "radian",
            "autolimits": "true",
            "meshdir": str(Path(self.hrdf_path, "meshes"))
        }

    def add_mesh(self, prefix=None):
        asset_elem = self.get_elem("asset")
        for mesh, file_type in self.mesh_file_type.items():
            # check mesh file exists
            mesh_file = Path(self.hrdf_path, "meshes", f"{mesh}.{file_type}")
            assert mesh_file.exists(), f"Mesh file {mesh_file} does not exist"
            mesh_elem = ET.SubElement(asset_elem, 'mesh', attrib={"name": get_prefix_name(prefix, mesh), "file": f"{mesh}.{file_type}"})

    def add_actuator(self, prefix=None):
        actuator_elem = ET.SubElement(self.xml_root, 'actuator')
        for actuator_data in self.data_dict["actuator"]:
            motor_elem = ET.SubElement(actuator_elem, 'motor')
            motor_elem.set("name", get_prefix_name(prefix, dict2str(actuator_data, "name")))
            motor_elem.set("joint", get_prefix_name(prefix, dict2str(actuator_data, "joint")))
            motor_elem.set("ctrlrange", dict2str(actuator_data, "ctrlrange"))

    def generate(self, prefix=None):
        self.add_compiler()
        self.add_mesh(prefix=prefix)
        self.add_all_body(prefix=prefix)
        if "actuator" in self.data_dict:
            self.add_actuator(prefix=prefix)
