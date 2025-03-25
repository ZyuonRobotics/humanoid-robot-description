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

DEFAULT_GROUND_TEXTURE_ATTR = {
    "type": "2d",
    "name": "groundplane",
    "builtin": "checker",
    "mark": "edge",
    "rgb1": "0.2 0.3 0.4",
    "rgb2": "0.1 0.2 0.3",
    "markrgb": "0.8 0.8 0.8",
    "width": "300",
    "height": "300"
}
DEFAULT_GROUND_MATERIAL_ATTR ={
    "name": "groundplane",
    "texture": "groundplane",
    "texuniform": "true",
    "texrepeat": "5 5",
    "reflectance": "0.2"
}

DEFAULT_GROUND_GEOM_ATTR = {
    "name": "floor",
    "size": "0 0 0.05",
    "type": "plane",
    "material": "groundplane",
    "condim": "3",
    "conaffinity": "15"
}

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
        self.ground_dict = None

        self.xml_root = ET.Element('mujoco')

    def load(self):
        with open(os.path.join(self.ehdf_path, "meta.json"), "r") as f:
            meta_info = json.load(f)
        assert RobotFormatType(meta_info["format_type"]) == self.format_type, f"Format type mismatch"
        self.body_parent_id = meta_info["body_parent_id"]
        self.mesh_file_type = meta_info["mesh_file_type"]
        self.ground_dict = meta_info["ground"]

        self.data_dict = {}
        for name in ["body", "joint", "mesh", "collision", "actuator"]:
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

    def generate_all_body_xml(self, parent, current_index):
        for child_index, parent_idx in enumerate(self.body_parent_id):
            if child_index == parent_idx: # skip world body
                continue
            elif parent_idx == current_index:
                body_elem = self.generate_single_body_xml(parent, child_index)
                self.generate_all_body_xml(body_elem, child_index)

    def add_compiler(self):
        compiler_elem = ET.SubElement(self.xml_root, 'compiler', attrib={
            "angle": "radian",
            "autolimits": "true",
            "meshdir": os.path.join(self.ehdf_path, "meshes")
        })

    def add_visual(self):
        visual_elem = ET.SubElement(self.xml_root, 'visual')
        headlight_elem = ET.SubElement(visual_elem, 'headlight',
                                       attrib={"diffuse": "0.6 0.6 0.6", "ambient": "0.3 0.3 0.3", "specular": "0 0 0"})
        rgba_elem = ET.SubElement(visual_elem, 'rgba', attrib={"haze": "0.15 0.25 0.35 1"})
        global_elem = ET.SubElement(visual_elem, 'global', attrib={"azimuth": "160", "elevation": "-20"})

    def add_asset(self):
        asset_elem = ET.SubElement(self.xml_root, 'asset')

        ET.SubElement(asset_elem, "texture",
                      attrib={"type": "skybox", "builtin": "gradient", "rgb1": "0.3 0.5 0.7", "rgb2": "0 0 0",
                              "width": "512", "height": "3072"})
        ET.SubElement(asset_elem, "texture", attrib=DEFAULT_GROUND_TEXTURE_ATTR)
        ET.SubElement(asset_elem, "material", attrib=DEFAULT_GROUND_MATERIAL_ATTR)

        for mesh, file_type in self.mesh_file_type.items():
            mesh_elem = ET.SubElement(asset_elem, 'mesh',
                                      attrib={"name": mesh, "file": f"{mesh}.{file_type}"})

    def add_ground(self, worldbody_elem):
        ground_attr = DEFAULT_GROUND_GEOM_ATTR
        if self.ground_dict is not None:
            for key, value in self.ground_dict.items():
                ground_attr[key] = value
        geom_elem = ET.SubElement(worldbody_elem, 'geom', attrib=ground_attr)

    def add_worldbody(self):
        world_body = ET.SubElement(self.xml_root, 'worldbody')
        light_elem = ET.SubElement(world_body, "light",
                                   attrib={"pos": "0 0 3.5", "dir": "0 0 -1", "directional": "true"})
        self.add_ground(world_body)

        self.generate_all_body_xml(world_body, 0)

    def add_actuator(self):
        actuator_elem = ET.SubElement(self.xml_root, 'actuator')
        for actuator_data in self.data_dict["actuator"]:
            motor_elem = ET.SubElement(actuator_elem, 'motor')
            for key in ["name", "joint", "ctrlrange"]:
                motor_elem.set(key, dict2str(actuator_data, key))

    def generate(self):
        self.add_compiler()
        self.add_visual()
        self.add_asset()
        self.add_worldbody()
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