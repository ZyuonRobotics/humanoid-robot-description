import pytest
import xml.etree.ElementTree as ET
from unittest.mock import Mock, patch

from hurodes.generators.urdf_generator.urdf_generator_base import URDFGeneratorBase


class SampleURDFGenerator(URDFGeneratorBase):
    """Test implementation of URDFGeneratorBase for testing purposes."""
    
    def __init__(self, robot_name):
        super().__init__()
        self._robot_name = robot_name
    
    def _load(self):
        pass
    
    def _generate(self, prefix=None):
        # Create a simple test structure
        link = ET.SubElement(self.xml_root, 'link', name='test_link')
        joint = ET.SubElement(self.xml_root, 'joint', name='test_joint', type='revolute')
        ET.SubElement(joint, 'parent', link='base_link')
        ET.SubElement(joint, 'child', link='test_link')


class URDFGeneratorTestHelper(URDFGeneratorBase):
    """Helper class without constructor for pytest compatibility."""
    
    def load(self):
        pass
    
    def generate(self, prefix=None):
        # Create a simple test structure
        link = ET.SubElement(self.xml_root, 'link', name='test_link')
        joint = ET.SubElement(self.xml_root, 'joint', name='test_joint', type='revolute')
        ET.SubElement(joint, 'parent', link='base_link')
        ET.SubElement(joint, 'child', link='test_link')


class TestURDFGeneratorBase:
    """Test cases for URDFGeneratorBase class."""

    def test_init_with_robot_name(self):
        """Test initialization with robot name."""
        generator = SampleURDFGenerator("test_robot")
        assert generator.robot_name == "test_robot"

    def test_xml_root_creation(self):
        """Test XML root element creation with robot name."""
        generator = SampleURDFGenerator("test_robot")
        root = generator.xml_root
        assert root.tag == "robot"
        assert root.get("name") == "test_robot"

    def test_xml_root_singleton(self):
        """Test that xml_root returns the same instance."""
        generator = SampleURDFGenerator("test_robot")
        root1 = generator.xml_root
        root2 = generator.xml_root
        assert root1 is root2

    def test_urdf_str_property(self):
        """Test urdf_str property returns format_str."""
        generator = SampleURDFGenerator("test_robot")
        # Generate content first to set the XML root
        generator.load()
        generator.generate()
        # Since xml_str is inherited from GeneratorBase, test that it works
        urdf_content = generator.xml_str
        assert isinstance(urdf_content, str)
        assert 'robot' in urdf_content

    def test_all_link_names_empty(self):
        """Test all_link_names with no links."""
        generator = SampleURDFGenerator("test_robot")
        assert generator.all_link_names == []

    def test_all_link_names_with_links(self):
        """Test all_link_names with multiple links."""
        generator = SampleURDFGenerator("test_robot")
        generator.load()
        generator.generate()  # This adds test_link
        link_names = generator.all_link_names
        assert "test_link" in link_names

    def test_all_joint_names_empty(self):
        """Test all_joint_names with no joints."""
        generator = SampleURDFGenerator("test_robot")
        assert generator.all_joint_names == []

    def test_all_joint_names_with_joints(self):
        """Test all_joint_names with multiple joints."""
        generator = SampleURDFGenerator("test_robot")
        generator.load()
        generator.generate()  # This adds test_joint
        joint_names = generator.all_joint_names
        assert "test_joint" in joint_names

    def test_link_tree_str_single_root(self):
        """Test link_tree_str with single root link."""
        generator = SampleURDFGenerator("test_robot")
        # Add a root link that is not child of any joint
        ET.SubElement(generator.xml_root, 'link', name='root_link')
        tree_str = generator.link_tree_str
        assert "root_link" in tree_str

    def test_link_tree_str_multiple_roots(self):
        """Test link_tree_str with multiple root links."""
        generator = SampleURDFGenerator("test_robot")
        # Add multiple root links
        ET.SubElement(generator.xml_root, 'link', name='root1')
        ET.SubElement(generator.xml_root, 'link', name='root2')
        tree_str = generator.link_tree_str
        assert "Multiple root links found" in tree_str
        assert "root1" in tree_str
        assert "root2" in tree_str

    def test_link_tree_str_with_parent_child_relationship(self):
        """Test link_tree_str identifies root correctly when joints exist."""
        generator = SampleURDFGenerator("test_robot")
        # Add links
        ET.SubElement(generator.xml_root, 'link', name='parent_link')
        ET.SubElement(generator.xml_root, 'link', name='child_link')
        # Add joint that makes child_link a child
        joint = ET.SubElement(generator.xml_root, 'joint', name='test_joint')
        ET.SubElement(joint, 'parent', link='parent_link')
        ET.SubElement(joint, 'child', link='child_link')
        
        tree_str = generator.link_tree_str
        # child_link should not be considered a root since it's a child in a joint
        assert "parent_link" in tree_str

    def test_all_link_names_assertion_error_with_none_name(self):
        """Test that all_link_names raises assertion error when link has None name."""
        generator = SampleURDFGenerator("test_robot")
        # Add a link without name attribute
        ET.SubElement(generator.xml_root, 'link')
        
        with pytest.raises(AssertionError, match="None link name found"):
            generator.all_link_names

    def test_all_joint_names_assertion_error_with_none_name(self):
        """Test that all_joint_names raises assertion error when joint has None name."""
        generator = SampleURDFGenerator("test_robot")
        # Add a joint without name attribute
        ET.SubElement(generator.xml_root, 'joint')
        
        with pytest.raises(AssertionError, match="None joint name found"):
            generator.all_joint_names 