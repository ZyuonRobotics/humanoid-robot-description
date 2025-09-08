from abc import ABC, abstractmethod
import xml.etree.ElementTree as ET
from pathlib import Path

from hurodes.generators.generator_base import GeneratorBase
from hurodes.utils.string import get_elem_tree_str


class URDFGeneratorBase(GeneratorBase):
    """
    Base class for URDF (Unified Robot Description Format) generators.
    
    This class extends GeneratorBase with URDF-specific functionality
    including robot naming and URDF-specific XML structure.
    """
    
    def __init__(self):
        """
        Initialize URDF generator base class.
        
        The robot name will be set during the load method based on the directory name.
        """
        super().__init__()
        self._robot_name = None

    @property
    def robot_name(self) -> str:
        assert self._robot_name is not None, "Robot name not set"
        return self._robot_name

    def _xml_root_init(self):
        """
        Initialize the XML root element.
        """
        self._xml_root = ET.Element('robot', name=self.robot_name)

    def _clean(self):
        pass

    @property
    def all_link_names(self):
        """Get all link names in the URDF."""
        link_list = [elem.get("name") for elem in self.xml_root.findall(".//link")]
        assert None not in link_list, "None link name found"
        return link_list

    @property
    def all_joint_names(self):
        """Get all joint names in the URDF."""
        joint_list = [elem.get("name") for elem in self.xml_root.findall(".//joint")]
        assert None not in joint_list, "None joint name found"
        return joint_list

    @property
    def link_tree_str(self):
        """Get a string representation of the link tree structure."""
        # Find the root link (one that is not a child of any joint)
        child_links = set()
        for joint in self.xml_root.findall("joint"):
            child_elem = joint.find("child")
            if child_elem is not None:
                child_links.add(child_elem.get("link"))
        
        root_links = []
        for link in self.xml_root.findall("link"):
            if link.get("name") not in child_links:
                root_links.append(link)
        
        if len(root_links) == 1:
            return get_elem_tree_str(root_links[0], colorful=False)
        else:
            return f"Multiple root links found: {[link.get('name') for link in root_links]}" 
