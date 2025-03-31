import os
import xml.etree.ElementTree as ET
import json
import math
from collections import defaultdict
from copy import deepcopy

from colorama import Fore, Style
import numpy as np
import pandas as pd

from hurodes.mjcf_generator.generator_base import MJCFGeneratorBase
from hurodes.contants import RobotFormatType


def dict2str(data, name):
    keys_in_dict = [key for key in data.keys() if key.startswith(name)]
    assert len(keys_in_dict) > 0, f"No key starts with {name} in data: {data}"
    if len(keys_in_dict) == 1:
        assert keys_in_dict[0] == name, f"keys_in_dict: {keys_in_dict}"
        return str(data[name])
    else:
        for i in range(len(keys_in_dict)):
            assert f"{name}{i}" in keys_in_dict, f"{name}{i} not in keys_in_dict: {keys_in_dict}"
        return " ".join([str(data[f"{name}{i}"]) for i in range(len(keys_in_dict))])

def find_by_body_id(all_data, body_id):
    res = []
    for data in all_data:
        if data["bodyid"] == body_id:
            data_copy = deepcopy(data)
            del data_copy["bodyid"]
            res.append(data_copy)
    return res


class UnifiedMJCFGenerator(MJCFGeneratorBase):
    format_type = RobotFormatType.UNKNOWN

    def __init__(
            self,
            ehdf_path,
            disable_gravity=False,
            timestep=0.001
    ):
        super().__init__(disable_gravity=disable_gravity, timestep=timestep)
        self.ehdf_path = ehdf_path

        self.body_parent_id = None
        self.data_dict = None
        self.mesh_file_type = None

    def load(self):
        with open(os.path.join(self.ehdf_path, "meta.json"), "r") as f:
            meta_info = json.load(f)
        assert RobotFormatType(meta_info["format_type"]) == self.format_type, f"Format type mismatch"
        self.body_parent_id = meta_info["body_parent_id"]
        self.mesh_file_type = meta_info["mesh_file_type"]
        self.ground_dict = meta_info["ground"]

        self.data_dict = {}
        for name in ["body", "joint", "mesh", "collision", "actuator"]:
            if os.path.exists(os.path.join(self.ehdf_path, f"{name}.csv")):
                self.data_dict[name] = pd.read_csv(os.path.join(self.ehdf_path, f"{name}.csv")).to_dict("records")

    def generate_single_body_xml(self, parent_node, body_idx):
        # body element
        body_data = self.data_dict["body"][body_idx]
        body_elem = ET.SubElement(parent_node, 'body')
        for key in ["name", "pos", "quat"]:
            body_elem.set(key, dict2str(body_data, key))

        # inertial element
        inertial_elem = ET.SubElement(body_elem, 'inertial')
        for key_xml, key_mj in zip(["mass", "pos", "quat", "diaginertia"], ["mass", "ipos", "iquat", "inertia"]):
            inertial_elem.set(key_xml, dict2str(body_data, key_mj))

        # joint element
        joint_data_list = find_by_body_id(self.data_dict["joint"], body_idx)
        assert len(joint_data_list) == 1
        joint_elem = ET.SubElement(body_elem, 'joint')
        for key in ["name", "type", "pos", "axis", "range", "damping", "stiffness", "armature", "frictionloss"]:
            joint_elem.set(key, dict2str(joint_data_list[0], key))

        # mesh element
        mesh_data_list = find_by_body_id(self.data_dict["mesh"], body_idx)
        for mesh_data in mesh_data_list:
            mesh_elem = ET.SubElement(body_elem, 'geom')
            for key in ["type", "mesh", "contype", "conaffinity", "pos", "quat", "rgba"]:
                mesh_elem.set(key, dict2str(mesh_data, key))

        # collision element
        collision_data_list = find_by_body_id(self.data_dict["collision"], body_idx)
        for collision_data in collision_data_list:
            collision_elem = ET.SubElement(body_elem, 'geom')
            collision_elem.set("rgba", "0 0.7 0.3 0.1")
            for key in ["type", "pos", "quat", "size", "contype", "conaffinity", "friction"]:
                collision_elem.set(key, dict2str(collision_data, key))

        return body_elem

    def add_all_body(self, parent=None, current_index=0):
        if parent is None:
            parent = self.get_elem("worldbody")
        for child_index, parent_idx in enumerate(self.body_parent_id):
            if child_index == parent_idx: # skip world body
                continue
            elif parent_idx == current_index:
                body_elem = self.generate_single_body_xml(parent, child_index)
                self.add_all_body(body_elem, child_index)

    def add_compiler(self):
        self.get_elem("compiler").attrib = {
            "angle": "radian",
            "autolimits": "true",
            "meshdir": os.path.join(self.ehdf_path, "meshes")
        }

    def add_mesh(self):
        asset_elem = self.get_elem("asset")

        for mesh, file_type in self.mesh_file_type.items():
            mesh_elem = ET.SubElement(asset_elem, 'mesh', attrib={"name": mesh, "file": f"{mesh}.{file_type}"})

    def add_actuator(self):
        actuator_elem = ET.SubElement(self.xml_root, 'actuator')
        for actuator_data in self.data_dict["actuator"]:
            motor_elem = ET.SubElement(actuator_elem, 'motor')
            for key in ["name", "joint", "ctrlrange"]:
                motor_elem.set(key, dict2str(actuator_data, key))

    def generate(self):
        self.add_compiler()
        self.add_mesh()
        self.add_all_body()
        if "actuator" in self.data_dict:
            self.add_actuator()


if __name__ == '__main__':
    from hurodes import MJCF_ROBOTS_PATH, ROBOTS_PATH
    import mujoco
    import mujoco.viewer

    ehdf_path = os.path.join(ROBOTS_PATH, "kuavo_s45")
    generator = UnifiedMJCFGenerator(ehdf_path)
    xml_string = generator.export(os.path.join(ehdf_path, "robot.xml"))

    m = mujoco.MjModel.from_xml_string(xml_string)
    d = mujoco.MjData(m)
    with mujoco.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            mujoco.mj_step(m, d)
            viewer.sync()