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

from hurodes.generators.mjcf_generator.mjcf_generator_base import MJCFGeneratorBase
from hurodes.hrdf.hrdf import HumanoidRobot

def get_prefix_name(prefix, name):
    return f"{prefix}_{name}" if prefix else name

class HumanoidMJCFGenerator(MJCFGeneratorBase):
    def __init__(
            self,
            hrdf_path,
            disable_gravity=False,
            timestep=0.001
    ):
        super().__init__(disable_gravity=disable_gravity, timestep=timestep)
        self.hrdf_path = hrdf_path

        self.humanoid_robot = None

    def load(self):
        self.humanoid_robot = HumanoidRobot.from_dir(self.hrdf_path)

    def generate_single_body_xml(self, parent_body, body_idx, prefix=None):
        body_info = self.humanoid_robot.info_list["body"][body_idx]
        body_name = body_info["name"].data
        body_elem = ET.SubElement(parent_body, 'body', attrib=body_info.to_mujoco_dict("body"))
        inertial_elem = ET.SubElement(body_elem, 'inertial', attrib=body_info.to_mujoco_dict("inertial"))

        if parent_body.tag == "worldbody":
            joint_elem = ET.SubElement(body_elem, 'freejoint')
        else:
            joint_info = self.humanoid_robot.find_info_by_attr("body_name", body_name, "joint", single=True)
            joint_elem = ET.SubElement(body_elem, 'joint', attrib=joint_info.to_mujoco_dict())

        mesh_info_list = self.humanoid_robot.find_info_by_attr("body_name", body_name, "mesh")
        if mesh_info_list:
            for mesh_info in mesh_info_list:
                mesh_elem = ET.SubElement(body_elem, 'geom', attrib=mesh_info.to_mujoco_dict())

        simple_geom_info_list = self.humanoid_robot.find_info_by_attr("body_name", body_name, "simple_geom")
        if simple_geom_info_list:
            for simple_geom_info in simple_geom_info_list:
                simple_geom_elem = ET.SubElement(body_elem, 'geom', attrib=simple_geom_info.to_mujoco_dict())
        return body_elem

    def recursive_generate_body(self, parent=None, current_index=-1, prefix=None):
        if parent is None:
            parent = self.get_elem("worldbody")

        for child_index, parent_idx in enumerate(self.humanoid_robot.body_parent_id):
            if parent_idx == current_index:
                body_elem = self.generate_single_body_xml(parent, child_index, prefix=prefix)
                self.recursive_generate_body(body_elem, child_index, prefix=prefix)

    def add_compiler(self):
        self.get_elem("compiler").attrib = {
            "angle": "radian",
            "autolimits": "true",
            "meshdir": str(Path(self.hrdf_path, "meshes"))
        }
    
    def add_default(self):
        default_elem = self.get_elem("default")
        ET.SubElement(default_elem, "joint", attrib={"limited": "true"})
        ET.SubElement(default_elem, "motor", attrib={"ctrllimited": "true"})

    def add_mesh(self, prefix=None):
        asset_elem = self.get_elem("asset")
        mesh_name_set = set()
        for mesh_info in self.humanoid_robot.info_list["mesh"]:
            mesh_name = mesh_info["name"].data
            if mesh_name in mesh_name_set:
                continue

            mesh_file = Path(self.hrdf_path, "meshes", f"{mesh_name}.{self.humanoid_robot.mesh_file_type}")
            assert mesh_file.exists(), f"Mesh file {mesh_file} does not exist"
            mesh_elem = ET.SubElement(asset_elem, 'mesh', attrib={"name": get_prefix_name(prefix, mesh_name), "file": f"{mesh_name}.{self.humanoid_robot.mesh_file_type}"})
            mesh_name_set.add(mesh_name)

    def add_actuator(self, prefix=None):
        if len(self.humanoid_robot.info_list["actuator"]) == 0:
            return
            
        actuator_elem = ET.SubElement(self.xml_root, 'actuator')
        
        for joint_info in self.humanoid_robot.info_list["joint"]:
            actuator_info = self.humanoid_robot.find_info_by_attr("joint_name", joint_info["name"].data, "actuator", single=True)
            motor_elem = ET.SubElement(actuator_elem, 'motor', attrib=actuator_info.to_mujoco_dict())

    def generate(self, prefix=None):
        self.add_compiler()
        self.add_default()
        self.add_mesh(prefix=prefix)
        self.recursive_generate_body(prefix=prefix)
        self.add_actuator(prefix=prefix)
