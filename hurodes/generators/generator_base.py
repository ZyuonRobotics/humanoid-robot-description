from abc import ABC, abstractmethod
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional

from hurodes.utils.string import get_elem_tree_str


class GeneratorBase(ABC):
    """
    Base generator class for robot description formats.
    
    This abstract class provides common functionality shared between
    MJCF and URDF generators, including XML tree management, build workflow,
    and file export capabilities.
    """
    
    def __init__(self):
        """Initialize the base generator."""
        self._xml_root: Optional[ET.Element] = None
        self.ground_dict: Optional[dict] = None
    
    @property
    @abstractmethod
    def xml_root(self) -> ET.Element:
        """
        Get or create the root XML element.
        
        This property must be implemented by subclasses to create
        format-specific root elements (e.g., 'mujoco' for MJCF, 'robot' for URDF).
        
        Returns:
            The root XML element for the specific format
        """
        raise NotImplementedError("xml_root property must be implemented by subclasses")
    
    def destroy(self):
        """Clean up the XML tree by resetting the root element to None."""
        self._xml_root = None
    
    def get_elem(self, elem_name: str) -> ET.Element:
        """
        Get or create a top-level element under the root.
        
        If the element already exists, return it. If it doesn't exist,
        create it as a child of the root element.
        
        Args:
            elem_name: Name of the element to get or create
            
        Returns:
            The requested XML element
            
        Raises:
            AssertionError: If multiple elements with the same name are found
        """
        elems = self.xml_root.findall(elem_name)
        assert len(elems) <= 1, f"Multiple {elem_name} elements found"
        if len(elems) == 1:
            return elems[0]
        else:
            return ET.SubElement(self.xml_root, elem_name)
    
    @property
    def format_str(self) -> str:
        """
        Get the formatted XML string representation.
        
        This property must be implemented by subclasses to provide
        format-specific string output with appropriate headers and formatting.
        
        Returns:
            The formatted XML string for the specific format
        """
        assert self._xml_root is not None, "XML root is not set"
        xml_str = '<?xml version="1.0" encoding="utf-8"?>\n'
        tree = ET.ElementTree(self.xml_root)
        ET.indent(tree, space="  ", level=0)
        xml_str += ET.tostring(self.xml_root, encoding='unicode', method='xml')
        return xml_str
    
    @abstractmethod
    def load(self):
        """
        Load data from source files.
        
        This method must be implemented by subclasses to load
        format-specific data from input files or data structures.
        """
        raise NotImplementedError("load method must be implemented by subclasses")
    
    @abstractmethod
    def generate(self, prefix: Optional[str] = None):
        """
        Generate the robot description content.
        
        This method must be implemented by subclasses to create
        the format-specific XML structure and content.
        
        Args:
            prefix: Optional prefix for naming elements
        """
        raise NotImplementedError("generate method must be implemented by subclasses")
    
    def build(self):
        """
        Build the complete robot description.
        
        This method orchestrates the build process by:
        1. Loading data from sources
        2. Cleaning up any existing XML tree
        3. Generating the new content
        
        Subclasses can override this method to add format-specific
        build steps (e.g., adding scenes for MJCF).
        """
        self.load()
        self.destroy()
        self.generate()
    
    def export(self, file_path: Optional[Path] = None) -> str:
        """
        Export the robot description to a file or return as string.
        
        Args:
            file_path: Optional path to save the output file
            
        Returns:
            The formatted string representation of the robot description
        """
        self.build()
        content = self.format_str
        
        if file_path is not None:
            file_path = Path(file_path)
            file_path.parent.mkdir(parents=True, exist_ok=True)
            with open(file_path, "w", encoding="utf-8") as f:
                f.write(content)
        
        return content
    
    @property
    def element_tree_str(self) -> str:
        """
        Get a string representation of the XML tree structure.
        
        This method provides a hierarchical view of the XML structure
        for debugging and visualization purposes.
        
        Returns:
            String representation of the XML tree structure
        """
        if self._xml_root is not None:
            return get_elem_tree_str(self._xml_root, colorful=False)
        return "No XML tree available" 