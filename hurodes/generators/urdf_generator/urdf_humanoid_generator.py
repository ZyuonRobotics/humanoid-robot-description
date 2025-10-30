from pathlib import Path
import xml.etree.ElementTree as ET

from hurodes.generators.urdf_generator.urdf_generator_base import URDFGeneratorBase
from hurodes.generators.hrdf_mixin import HRDFMixin


class URDFHumanoidGenerator(HRDFMixin, URDFGeneratorBase):
    """
    URDF generator for humanoid robots.
    
    This class combines HRDFMixin for common humanoid functionality
    with URDFGeneratorBase for URDF-specific XML generation.
    """
    
    def __init__(self):
        """
        Initialize URDF humanoid generator.
        
        Args:
            hrdf_path: Path to the HRDF directory
            robot_name: Name of the robot (defaults to directory name)
        """
        super().__init__()

    def _generate(self, add_mujoco_tag=False, relative_mesh_path=True):
        """
        Generate the complete URDF for the humanoid robot.
        
        This method creates links and joints from the HRDF data.
        """
        if add_mujoco_tag:
            self._generate_mujoco_elem(relative_mesh_path=relative_mesh_path)
        self._generate_links(add_mujoco_tag)
        self._generate_joints(add_mujoco_tag)
        
    def _generate_links(self, add_mujoco_tag=False) -> dict:
        """
        Generate all link elements.
        """
        body_info_list = self.info_list("body")

        if add_mujoco_tag:
            for body_info in body_info_list:
                if body_info["id"].data == 0:
                    link_elem = ET.SubElement(self.xml_root, "link", attrib={"name": "world"})
        
        for body_info in body_info_list:
            link_elem = ET.SubElement(self.xml_root, "link")
            body_name = body_info["name"].data

            # Add body info to link
            body_info.to_urdf_elem(link_elem, "link")

            # Add simple geometries
            simple_geom_infos = self.get_info_by_attr("body_name", body_name, "simple_geom")
            for simple_geom_info in simple_geom_infos:
                simple_geom_info.to_urdf_elem(link_elem)

            # Add mesh geometries
            mesh_infos = self.get_info_by_attr("body_name", body_name, "mesh")
            for mesh_info in mesh_infos:
                mesh_info.to_urdf_elem(link_elem)
                
    def _generate_joints(self, add_mujoco_tag=False) -> dict:
        """
        Generate all joint elements.
        """
        joint_info_list = self.info_list("joint")

        if add_mujoco_tag:
            for body_info in self.info_list("body"):
                if body_info["id"].data == 0:
                    joint_elem = ET.SubElement(
                        self.xml_root, 
                        "joint", 
                        attrib={"name": "freejoint", "type": "floating"}
                    )
                    ET.SubElement(joint_elem, "child", attrib={"link": body_info["name"].data})
                    ET.SubElement(joint_elem, "parent", attrib={"link": "world"})
        
        for joint_info in joint_info_list:
            joint_elem = ET.SubElement(self.xml_root, "joint")
            joint_name = joint_info["name"].data
            body_name = joint_info["body_name"].data

            # Add joint info
            joint_info.to_urdf_elem(joint_elem)

            # Add child body info to joint
            child_body_info = self.get_info_by_attr("name", body_name, "body", single=True)
            child_body_info.to_urdf_elem(joint_elem, "child")

            # Add parent body info to joint
            parent_body_id = self.body_parent_id[child_body_info["id"].data]
            parent_body_info = self.get_info_by_attr("id", parent_body_id, "body", single=True)
            parent_body_info.to_urdf_elem(joint_elem, "parent")

            # Add actuator info
            actuator_info = self.get_info_by_attr("joint_name", joint_name, "actuator", single=True)
            actuator_info.to_urdf_elem(joint_elem)

    def _generate_mujoco_elem(self, relative_mesh_path=True):
        """
        Generate the MuJoCo element.
        """
        mujoco_elem = ET.SubElement(self.xml_root, "mujoco")
        ET.SubElement(mujoco_elem, 'compiler', attrib={
            'meshdir': "../meshes" if relative_mesh_path else str(self.mesh_directory),
            "balanceinertia": "true",
            "discardvisual": "false"
        })
