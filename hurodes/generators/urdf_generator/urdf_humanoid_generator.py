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

    def _generate(self, **kwargs):
        """
        Generate the complete URDF for the humanoid robot.
        
        This method creates links and joints from the HRDF data.
        """
        self._generate_links()
        self._generate_joints()
        
    def _generate_links(self) -> dict:
        """
        Generate all link elements.
        
        Returns:
            Dictionary mapping body names to link elements
        """
        link_dict = {}
        body_info_list = self.info_list("body")
        
        for body_info in body_info_list:
            link_elem = ET.SubElement(self.xml_root, "link")
            body_name = body_info["name"].data
            link_dict[body_name] = link_elem

            # Add body info to link
            body_info.to_urdf_elem(link_elem, "link")

            # Add simple geometries
            simple_geom_infos = self.find_info_by_attr("body_name", body_name, "simple_geom")
            for simple_geom_info in simple_geom_infos:
                simple_geom_info.to_urdf_elem(link_elem)

            # Add mesh geometries
            mesh_infos = self.find_info_by_attr("body_name", body_name, "mesh")
            for mesh_info in mesh_infos:
                mesh_info.to_urdf_elem(link_elem)
                
        return link_dict

    def _generate_joints(self) -> dict:
        """
        Generate all joint elements.
        
        Returns:
            Dictionary mapping joint names to joint elements
        """
        joint_dict = {}
        joint_info_list = self.info_list("joint")
        
        for joint_info in joint_info_list:
            joint_elem = ET.SubElement(self.xml_root, "joint")
            joint_name = joint_info["name"].data
            body_name = joint_info["body_name"].data
            joint_dict[joint_name] = joint_elem

            # Add joint info
            joint_info.to_urdf_elem(joint_elem)

            # Add body info to joint
            body_info = self.find_info_by_attr("name", body_name, "body", single=True)
            body_info.to_urdf_elem(joint_elem, "joint")

            # Add actuator info
            actuator_info = self.find_info_by_attr("joint_name", joint_name, "actuator", single=True)
            actuator_info.to_urdf_elem(joint_elem)
            
        return joint_dict
