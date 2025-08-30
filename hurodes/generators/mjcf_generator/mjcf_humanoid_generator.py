# import os
from pathlib import Path
import xml.etree.ElementTree as ET
import math
from collections import defaultdict
from copy import deepcopy

from colorama import Fore, Style
import numpy as np
import pandas as pd

from hurodes.generators.mjcf_generator.mjcf_generator_base import MJCFGeneratorBase
from hurodes.generators.hrdf_mixin import HRDFMixin
from hurodes.hrdf.hrdf import HRDF
from hurodes.utils.string import get_prefix_name


class MJCFHumanoidGenerator(HRDFMixin, MJCFGeneratorBase):
    """
    MJCF generator for humanoid robots.
    
    This class combines HRDFMixin for common humanoid functionality
    with MJCFGeneratorBase for MJCF-specific XML generation.
    """
    
    def __init__(self):
        super().__init__()

    def generate_single_body_xml(self, parent_body, body_idx, prefix=None):
        """
        Generate XML for a single body element.
        
        Args:
            parent_body: Parent XML element to attach the body to
            body_idx: Index of the body in the info list
            prefix: Optional prefix for naming elements
            
        Returns:
            The created body XML element
        """
        body_info_list = self.info_list("body")
        body_info = body_info_list[body_idx]
        body_name = body_info["name"].data
        
        # Create body element with attributes
        body_elem = ET.SubElement(parent_body, 'body', attrib=body_info.to_mujoco_dict("body", prefix=prefix))
        inertial_elem = ET.SubElement(body_elem, 'inertial', attrib=body_info.to_mujoco_dict("inertial", prefix=prefix))

        # Add joint (freejoint for root body, regular joint for others)
        if parent_body.tag == "worldbody":
            joint_elem = ET.SubElement(body_elem, 'freejoint')
        else:
            joint_info = self.find_info_by_attr("body_name", body_name, "joint", single=True)
            joint_elem = ET.SubElement(body_elem, 'joint', attrib=joint_info.to_mujoco_dict(prefix=prefix))

        # Add mesh geometries
        mesh_info_list = self.find_info_by_attr("body_name", body_name, "mesh")
        for mesh_info in mesh_info_list:
            mesh_elem = ET.SubElement(body_elem, 'geom', attrib=mesh_info.to_mujoco_dict(prefix=prefix))

        # Add simple geometries
        simple_geom_info_list = self.find_info_by_attr("body_name", body_name, "simple_geom")
        for simple_geom_info in simple_geom_info_list:
            simple_geom_elem = ET.SubElement(body_elem, 'geom', attrib=simple_geom_info.to_mujoco_dict(prefix=prefix))
            
        return body_elem

    def recursive_generate_body(self, parent=None, current_index=-1, prefix=None):
        """
        Recursively generate body elements in the XML tree.
        
        Args:
            parent: Parent XML element (defaults to worldbody)
            current_index: Current body index in the hierarchy
            prefix: Optional prefix for naming elements
        """
        if parent is None:
            parent = self.get_elem("worldbody")

        for child_index, parent_idx in enumerate(self.body_parent_id):
            if parent_idx == current_index:
                body_elem = self.generate_single_body_xml(parent, child_index, prefix=prefix)
                self.recursive_generate_body(body_elem, child_index, prefix=prefix)

    def add_compiler(self):
        """Add compiler configuration with mesh directory."""
        self.get_elem("compiler").attrib = {
            "angle": "radian",
            "autolimits": "true",
            "meshdir": str(self.mesh_directory)
        }
    
    def add_mesh(self, prefix=None):
        """
        Add mesh assets to the MJCF.
        
        Args:
            prefix: Optional prefix for mesh names
        """
        asset_elem = self.get_elem("asset")
        mesh_name_set = set()
        
        mesh_info_list = self.info_list("mesh")
        for mesh_info in mesh_info_list:
            mesh_name = mesh_info["name"].data
            if mesh_name in mesh_name_set:
                continue

            # Validate mesh file exists
            mesh_file = self.validate_mesh_exists(mesh_name)
            
            # Create mesh element
            mesh_elem = ET.SubElement(
                asset_elem, 
                'mesh', 
                attrib={
                    "name": get_prefix_name(prefix, mesh_name), 
                    "file": f"{mesh_name}.{self.mesh_file_type}"
                }
            )
            mesh_name_set.add(mesh_name)

    def add_actuator(self, prefix=None):
        """
        Add actuators for joints.
        
        Args:
            prefix: Optional prefix for actuator names
        """
        actuator_info_list = self.info_list("actuator")
        if len(actuator_info_list) == 0:
            return
            
        actuator_elem = ET.SubElement(self.xml_root, 'actuator')
        
        # Keep the order of joints
        joint_info_list = self.info_list("joint")
        for joint_info in joint_info_list:
            actuator_info = self.find_info_by_attr("joint_name", joint_info["name"].data, "actuator", single=True)
            motor_elem = ET.SubElement(actuator_elem, 'motor', attrib=actuator_info.to_mujoco_dict(prefix=prefix))

    def generate(self, prefix=None):
        """
        Generate the complete MJCF for the humanoid robot.
        
        Args:
            prefix: Optional prefix for element names
        """
        self.add_compiler()
        self.add_mesh(prefix=prefix)
        self.recursive_generate_body(prefix=prefix)
        self.add_actuator(prefix=prefix)
