from pathlib import Path
import xml.etree.ElementTree as ET
import json
import math
from collections import defaultdict
from copy import deepcopy

from colorama import Fore, Style
import numpy as np
import pandas as pd

from hurodes.urdf_generator.generator_base import URDFGeneratorBase
from hurodes.hrdf.base.info import InfoBase, load_csv, find_info_by_attr
from hurodes.hrdf import INFO_DICT

def get_prefix_name(prefix, name):
    return f"{prefix}_{name}" if prefix else name

class UnifiedURDFGenerator(URDFGeneratorBase):
    def __init__(
            self,
            hrdf_path,
            robot_name=None
    ):
        # Use directory name as robot name if not specified
        if robot_name is None:
            robot_name = Path(hrdf_path).name
        super().__init__(robot_name)
        self.hrdf_path = hrdf_path

        self.body_parent_id: list[int] = []
        self.body_info_list: list[InfoBase] = []
        self.joint_info_list: list[InfoBase] = []
        self.actuator_info_list: list[InfoBase] = []
        self.mesh_info_list: list[InfoBase] = []
        self.simple_geom_info_list: list[InfoBase] = []
        self.ground_dict: dict[str, InfoBase] = {}
        self.mesh_file_type = None

    def load(self):
        with open(Path(self.hrdf_path, "meta.json"), "r") as f:
            meta_info = json.load(f)
        self.body_parent_id = meta_info["body_parent_id"]
        self.mesh_file_type = meta_info["mesh_file_type"]
        self.ground_dict = meta_info["ground"]
    
        for name in ["body", "joint", "actuator", "mesh", "simple_geom"]:
            component_csv = Path(self.hrdf_path, f"{name}.csv")
            if component_csv.exists():
                setattr(self, f"{name}_info_list", load_csv(str(component_csv), INFO_DICT[name]))

    def generate(self):
        link_dict = {}
        for body_info in self.body_info_list:
            link_elem = ET.SubElement(self.xml_root, "link")
            link_dict[body_info["name"].data] = link_elem

            body_info.to_urdf_elem(link_elem, "link")

            simple_geom_infos = find_info_by_attr("body_name", body_info["name"].data, self.simple_geom_info_list)
            for simple_geom_info in simple_geom_infos:
                simple_geom_info.to_urdf_elem(link_elem)

            mesh_infos = find_info_by_attr("body_name", body_info["name"].data, self.mesh_info_list)
            for mesh_info in mesh_infos:
                mesh_info.to_urdf_elem(link_elem)

        joint_dict = {}
        for joint_info in self.joint_info_list:
            joint_elem = ET.SubElement(self.xml_root, "joint")
            joint_dict[joint_info["name"].data] = joint_elem

            joint_info.to_urdf_elem(joint_elem)

            body_info = find_info_by_attr("name", joint_info["body_name"].data, self.body_info_list, return_one=True)
            body_info.to_urdf_elem(joint_elem, "joint")

            actuator_info = find_info_by_attr("joint_name", joint_info["name"].data, self.actuator_info_list, return_one=True)
            actuator_info.to_urdf_elem(joint_elem)
