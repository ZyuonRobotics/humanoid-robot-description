from pathlib import Path
import xml.etree.ElementTree as ET
import json
import math
from collections import defaultdict
from copy import deepcopy

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R

from hurodes.urdf_generator.generator_base import URDFGeneratorBase
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
    """Add prefix to name if prefix is provided."""
    return f"{prefix}_{name}" if prefix else name


def euler_to_rpy(euler_str):
    """Convert euler angles to URDF RPY format."""
    if not euler_str or euler_str.strip() == "":
        return "0 0 0"
    
    # Parse euler angles (assuming they are in radians)
    angles = [float(x) for x in euler_str.split()]
    if len(angles) == 3:
        # URDF uses roll, pitch, yaw (RPY) format
        return f"{angles[0]} {angles[1]} {angles[2]}"
    return "0 0 0"


def quat_to_rpy(quat_str):
    """Convert quaternion to URDF RPY format using scipy."""
    if not quat_str or quat_str.strip() == "":
        return "0 0 0"
    
    # Parse quaternion (w, x, y, z)
    quat = [float(x) for x in quat_str.split()]
    if len(quat) == 4:
        w, x, y, z = quat
        
        # Create rotation object from quaternion (scipy expects [x, y, z, w] order)
        rotation = R.from_quat([x, y, z, w])
        
        # Convert to Euler angles in XYZ order (roll, pitch, yaw)
        euler_angles = rotation.as_euler('xyz', degrees=False)
        roll, pitch, yaw = euler_angles
        
        return f"{roll} {pitch} {yaw}"
    
    return "0 0 0"


class UnifiedURDFGenerator(URDFGeneratorBase):
    def __init__(self, hrdf_path):
        """
        Initialize the unified URDF generator.
        
        Args:
            hrdf_path: Path to the HRDF data directory
        """
        super().__init__(Path(hrdf_path).name)
        self.hrdf_path = hrdf_path

        self.body_parent_id: list[int] = []
        self.data_dict: dict[str, list[dict]] = {}
        self.mesh_file_type: dict[str, str] = {}

    def load(self):
        """Load data from HRDF files."""
        
        with open(Path(self.hrdf_path, "meta.json"), "r") as f:
            meta_info = json.load(f)
        self.body_parent_id = meta_info["body_parent_id"]
        self.mesh_file_type = meta_info["mesh_file_type"]

        self.data_dict = {}
        for name in ["body", "joint", "mesh", "collision"]:
            component_csv = Path(self.hrdf_path, f"{name}.csv")
            if component_csv.exists():
                self.data_dict[name] = pd.read_csv(component_csv).to_dict("records")

    def generate_single_link_xml(self, body_idx, prefix=None):
        """
        Generate URDF link element for a single body.
        
        Args:
            body_idx: Index of the body in the data
            prefix: Optional prefix for names
            
        Returns:
            ET.Element: The link element
        """
        body_data = self.data_dict["body"][body_idx]
        link_name = get_prefix_name(prefix, dict2str(body_data, "name"))
        
        # Create link element
        link_elem = ET.Element('link', name=link_name)
        
        # Add inertial properties
        inertial_elem = ET.SubElement(link_elem, 'inertial')
        
        # Mass
        mass_elem = ET.SubElement(inertial_elem, 'mass')
        mass_elem.set('value', dict2str(body_data, "mass"))
        
        # Center of mass origin
        origin_elem = ET.SubElement(inertial_elem, 'origin')
        try:
            ipos = dict2str(body_data, "ipos")
        except (KeyError, AssertionError):
            ipos = "0 0 0"
        origin_elem.set('xyz', ipos)
        
        # Convert quaternion to RPY if available
        try:
            iquat = dict2str(body_data, "iquat")
        except (KeyError, AssertionError):
            iquat = ""
        if iquat:
            rpy = quat_to_rpy(iquat)
            origin_elem.set('rpy', rpy)
        else:
            origin_elem.set('rpy', '0 0 0')
        
        # Inertia tensor
        inertia_elem = ET.SubElement(inertial_elem, 'inertia')
        try:
            inertia_str = dict2str(body_data, "inertia")
        except (KeyError, AssertionError):
            inertia_str = "0 0 0"
        inertia_values = inertia_str.split()
        if len(inertia_values) >= 3:
            # Assuming diagonal inertia matrix
            inertia_elem.set('ixx', inertia_values[0])
            inertia_elem.set('iyy', inertia_values[1])
            inertia_elem.set('izz', inertia_values[2])
            inertia_elem.set('ixy', '0')
            inertia_elem.set('ixz', '0')
            inertia_elem.set('iyz', '0')
        
        # Add visual elements from mesh data
        if "mesh" in self.data_dict:
            mesh_data_list = find_by_body_id(self.data_dict["mesh"], body_idx)
            for mesh_data in mesh_data_list:
                visual_elem = ET.SubElement(link_elem, 'visual')
                
                # Visual origin
                visual_origin = ET.SubElement(visual_elem, 'origin')
                try:
                    mesh_pos = dict2str(mesh_data, "pos")
                except (KeyError, AssertionError):
                    mesh_pos = "0 0 0"
                visual_origin.set('xyz', mesh_pos)
                
                try:
                    mesh_quat = dict2str(mesh_data, "quat")
                except (KeyError, AssertionError):
                    mesh_quat = ""
                if mesh_quat:
                    rpy = quat_to_rpy(mesh_quat)
                    visual_origin.set('rpy', rpy)
                else:
                    visual_origin.set('rpy', '0 0 0')
                
                # Geometry
                geometry_elem = ET.SubElement(visual_elem, 'geometry')
                mesh_elem = ET.SubElement(geometry_elem, 'mesh')
                mesh_name = dict2str(mesh_data, "mesh")
                mesh_file_type = self.mesh_file_type.get(mesh_name, "stl")
                mesh_elem.set('filename', f"meshes/{mesh_name}.{mesh_file_type}")
                
                # Material (color)
                try:
                    rgba = dict2str(mesh_data, "rgba")
                except (KeyError, AssertionError):
                    rgba = "0.8 0.8 0.8 1.0"
                rgba_values = rgba.split()
                if len(rgba_values) >= 3:
                    material_elem = ET.SubElement(visual_elem, 'material')
                    material_elem.set('name', f"{link_name}_material")
                    color_elem = ET.SubElement(material_elem, 'color')
                    # URDF uses RGBA format
                    color_elem.set('rgba', rgba)
        
        # Add collision elements
        if "collision" in self.data_dict:
            collision_data_list = find_by_body_id(self.data_dict["collision"], body_idx)
            for idx, collision_data in enumerate(collision_data_list):
                collision_elem = ET.SubElement(link_elem, 'collision')
                
                # Collision origin
                collision_origin = ET.SubElement(collision_elem, 'origin')
                try:
                    collision_pos = dict2str(collision_data, "pos")
                except (KeyError, AssertionError):
                    collision_pos = "0 0 0"
                collision_origin.set('xyz', collision_pos)
                
                try:
                    collision_quat = dict2str(collision_data, "quat")
                except (KeyError, AssertionError):
                    collision_quat = ""
                if collision_quat:
                    rpy = quat_to_rpy(collision_quat)
                    collision_origin.set('rpy', rpy)
                else:
                    collision_origin.set('rpy', '0 0 0')
                
                # Collision geometry
                geometry_elem = ET.SubElement(collision_elem, 'geometry')
                try:
                    collision_type = dict2str(collision_data, "type")
                except (KeyError, AssertionError):
                    collision_type = "box"
                
                if collision_type == "box":
                    box_elem = ET.SubElement(geometry_elem, 'box')
                    try:
                        size = dict2str(collision_data, "size")
                    except (KeyError, AssertionError):
                        size = "0.1 0.1 0.1"
                    box_elem.set('size', size)
                elif collision_type == "sphere":
                    sphere_elem = ET.SubElement(geometry_elem, 'sphere')
                    try:
                        size = dict2str(collision_data, "size")
                    except (KeyError, AssertionError):
                        size = "0.1"
                    radius = size.split()[0] if size else "0.1"
                    sphere_elem.set('radius', radius)
                elif collision_type == "cylinder":
                    cylinder_elem = ET.SubElement(geometry_elem, 'cylinder')
                    try:
                        size = dict2str(collision_data, "size")
                    except (KeyError, AssertionError):
                        size = "0.1 0.1"
                    size_parts = size.split()
                    radius = size_parts[0] if len(size_parts) > 0 else "0.1"
                    length = size_parts[1] if len(size_parts) > 1 else "0.1"
                    cylinder_elem.set('radius', radius)
                    cylinder_elem.set('length', length)
        
        return link_elem

    def generate_single_joint_xml(self, body_idx, prefix=None):
        """
        Generate URDF joint element for a single body.
        
        Args:
            body_idx: Index of the body in the data
            prefix: Optional prefix for names
            
        Returns:
            ET.Element: The joint element, or None if this is the root body
        """
        if body_idx == 0 or self.body_parent_id[body_idx] == body_idx:
            # This is the root body, no joint needed
            return None
        
        joint_data_list = find_by_body_id(self.data_dict["joint"], body_idx)
        if not joint_data_list:
            return None
        
        assert len(joint_data_list) == 1, f"Expected exactly one joint for body {body_idx}"
        joint_data = joint_data_list[0]
        
        # Get parent and child link names
        parent_body_idx = self.body_parent_id[body_idx]
        parent_body_data = self.data_dict["body"][parent_body_idx]
        child_body_data = self.data_dict["body"][body_idx]
        
        parent_link_name = get_prefix_name(prefix, dict2str(parent_body_data, "name"))
        child_link_name = get_prefix_name(prefix, dict2str(child_body_data, "name"))
        
        # Create joint element
        joint_name = get_prefix_name(prefix, dict2str(joint_data, "name"))
        try:
            joint_type = dict2str(joint_data, "type")
        except (KeyError, AssertionError):
            joint_type = "revolute"
        
        # Map MuJoCo joint types to URDF joint types
        type_mapping = {
            "hinge": "revolute",
            "slide": "prismatic",
            "ball": "continuous",
            "free": "floating"
        }
        urdf_joint_type = type_mapping.get(joint_type, joint_type)
        
        joint_elem = ET.Element('joint', name=joint_name, type=urdf_joint_type)
        
        # Parent and child links
        parent_elem = ET.SubElement(joint_elem, 'parent')
        parent_elem.set('link', parent_link_name)
        
        child_elem = ET.SubElement(joint_elem, 'child')
        child_elem.set('link', child_link_name)
        
        # Joint origin (position and orientation of child relative to parent)
        origin_elem = ET.SubElement(joint_elem, 'origin')
        try:
            joint_pos = dict2str(joint_data, "pos")
        except (KeyError, AssertionError):
            joint_pos = "0 0 0"
        origin_elem.set('xyz', joint_pos)
        
        # Convert quaternion to RPY for joint orientation
        try:
            joint_quat = dict2str(child_body_data, "quat")
        except (KeyError, AssertionError):
            joint_quat = ""
        if joint_quat:
            rpy = quat_to_rpy(joint_quat)
            origin_elem.set('rpy', rpy)
        else:
            origin_elem.set('rpy', '0 0 0')
        
        # Joint axis
        if urdf_joint_type in ["revolute", "prismatic", "continuous"]:
            axis_elem = ET.SubElement(joint_elem, 'axis')
            try:
                axis = dict2str(joint_data, "axis")
            except (KeyError, AssertionError):
                axis = "0 0 1"
            axis_elem.set('xyz', axis)
        
        # Joint limits
        if urdf_joint_type == "revolute":
            limit_elem = ET.SubElement(joint_elem, 'limit')
            try:
                joint_range = dict2str(joint_data, "range")
            except (KeyError, AssertionError):
                joint_range = "-1.57 1.57"
            range_parts = joint_range.split()
            if len(range_parts) >= 2:
                limit_elem.set('lower', range_parts[0])
                limit_elem.set('upper', range_parts[1])
            
            # Add effort and velocity limits if available
            limit_elem.set('effort', '100')  # Default effort limit
            limit_elem.set('velocity', '10')  # Default velocity limit
        
        # Joint dynamics
        dynamics_elem = ET.SubElement(joint_elem, 'dynamics')
        try:
            damping = dict2str(joint_data, "damping")
        except (KeyError, AssertionError):
            damping = "0"
        try:
            friction = dict2str(joint_data, "frictionloss")
        except (KeyError, AssertionError):
            friction = "0"
        dynamics_elem.set('damping', damping)
        dynamics_elem.set('friction', friction)
        
        return joint_elem

    def add_all_links_and_joints(self, prefix=None):
        """Add all links and joints to the URDF."""
        # First, add all links
        for body_idx in range(len(self.data_dict["body"])):
            link_elem = self.generate_single_link_xml(body_idx, prefix=prefix)
            self.xml_root.append(link_elem)
        
        # Then, add all joints
        for body_idx in range(len(self.data_dict["body"])):
            joint_elem = self.generate_single_joint_xml(body_idx, prefix=prefix)
            if joint_elem is not None:
                self.xml_root.append(joint_elem)

    def generate(self, prefix=None):
        """Generate the complete URDF."""
        self.add_all_links_and_joints(prefix=prefix) 

if __name__ == "__main__":
    from hurodes import ROBOTS_PATH
    hrdf_path = Path(ROBOTS_PATH, "zhaplin_v0")

    generator = UnifiedURDFGenerator(hrdf_path)
    generator.load()
    generator.generate()
    print(generator.format_str)