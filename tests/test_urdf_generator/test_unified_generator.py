import pytest
import xml.etree.ElementTree as ET
import tempfile
import json
import os
from pathlib import Path
from unittest.mock import Mock, patch, MagicMock

from hurodes.urdf_generator.unified_generator import (
    UnifiedURDFGenerator, 
    get_prefix_name, 
    find_info_by_attr
)
from hurodes.hrdf.base.info import InfoBase


class TestHelperFunctions:
    """Test cases for helper functions."""

    def test_get_prefix_name_with_prefix(self):
        """Test get_prefix_name with prefix."""
        result = get_prefix_name("robot1", "arm")
        assert result == "robot1_arm"

    def test_get_prefix_name_without_prefix(self):
        """Test get_prefix_name without prefix."""
        result = get_prefix_name(None, "arm")
        assert result == "arm"
        
        result = get_prefix_name("", "arm")
        assert result == "arm"

    def test_find_info_by_attr_single_match(self):
        """Test find_info_by_attr with single match."""
        # Create mock info objects
        info1 = Mock()
        info1.__getitem__ = Mock(return_value=Mock(data="value1"))
        info2 = Mock()
        info2.__getitem__ = Mock(return_value=Mock(data="value2"))
        info3 = Mock()
        info3.__getitem__ = Mock(return_value=Mock(data="value1"))

        info_list = [info1, info2, info3]
        
        # Test return_one=True
        result = find_info_by_attr("attr_name", "value2", info_list, return_one=True)
        assert result == info2
        
        # Test return_one=False
        results = find_info_by_attr("attr_name", "value1", info_list, return_one=False)
        assert len(results) == 2
        assert info1 in results
        assert info3 in results

    def test_find_info_by_attr_multiple_matches_assertion_error(self):
        """Test find_info_by_attr raises assertion error when multiple matches with return_one=True."""
        info1 = Mock()
        info1.__getitem__ = Mock(return_value=Mock(data="value1"))
        info2 = Mock()
        info2.__getitem__ = Mock(return_value=Mock(data="value1"))

        info_list = [info1, info2]
        
        with pytest.raises(AssertionError, match="Found multiple info"):
            find_info_by_attr("attr_name", "value1", info_list, return_one=True)

    def test_find_info_by_attr_no_matches(self):
        """Test find_info_by_attr with no matches."""
        info1 = Mock()
        info1.__getitem__ = Mock(return_value=Mock(data="value1"))
        
        info_list = [info1]
        
        results = find_info_by_attr("attr_name", "nonexistent", info_list, return_one=False)
        assert results == []


class TestUnifiedURDFGenerator:
    """Test cases for UnifiedURDFGenerator class."""

    @pytest.fixture
    def temp_hrdf_dir(self):
        """Create a temporary HRDF directory structure for testing."""
        with tempfile.TemporaryDirectory() as temp_dir:
            # Create meta.json
            meta_data = {
                "body_parent_id": [-1, 0, 0],
                "mesh_file_type": "obj",
                "ground": {"name": "ground", "type": "plane"}
            }
            with open(Path(temp_dir, "meta.json"), "w") as f:
                json.dump(meta_data, f)
            
            # Create CSV files directory structure
            yield temp_dir

    @pytest.fixture
    def mock_info_lists(self):
        """Create mock info lists for testing."""
        # Mock body info
        body_info1 = Mock()
        body_info1.__getitem__ = Mock(side_effect=lambda key: Mock(data="base_link" if key == "name" else ""))
        body_info1.to_urdf_dict = Mock(return_value={"name": "base_link"})
        
        body_info2 = Mock()
        body_info2.__getitem__ = Mock(side_effect=lambda key: Mock(data="arm_link" if key == "name" else ""))
        body_info2.to_urdf_dict = Mock(return_value={"name": "arm_link"})
        
        # Mock joint info  
        joint_info = Mock()
        joint_info.__getitem__ = Mock(side_effect=lambda key: 
            Mock(data="arm_joint" if key == "name" else 
                      "arm_link" if key == "body_name" else ""))
        joint_info.to_urdf_dict = Mock(return_value={"name": "arm_joint", "type": "revolute"})
        
        # Mock mesh info
        mesh_info = Mock()
        mesh_info.__getitem__ = Mock(side_effect=lambda key: Mock(data="base_link" if key == "body_name" else "test_mesh"))
        mesh_info.to_urdf_dict = Mock(return_value={"filename": "test.obj"})
        
        return {
            "body": [body_info1, body_info2],
            "joint": [joint_info],
            "mesh": [mesh_info],
            "actuator": [],
            "simple_geom": []
        }

    def test_init_with_robot_name(self, temp_hrdf_dir):
        """Test initialization with explicit robot name."""
        generator = UnifiedURDFGenerator(temp_hrdf_dir, robot_name="test_robot")
        assert generator.robot_name == "test_robot"
        assert generator.hrdf_path == temp_hrdf_dir

    def test_init_without_robot_name(self, temp_hrdf_dir):
        """Test initialization with robot name from directory."""
        generator = UnifiedURDFGenerator(temp_hrdf_dir)
        expected_name = Path(temp_hrdf_dir).name
        assert generator.robot_name == expected_name

    def test_load_meta_json(self, temp_hrdf_dir):
        """Test loading meta.json file."""
        generator = UnifiedURDFGenerator(temp_hrdf_dir)
        generator.load()
        
        assert generator.body_parent_id == [-1, 0, 0]
        assert generator.mesh_file_type == "obj"
        assert generator.ground_dict["name"] == "ground"

    @patch('hurodes.urdf_generator.unified_generator.load_csv')
    def test_load_csv_files(self, mock_load_csv, temp_hrdf_dir):
        """Test loading CSV files."""
        # Create CSV files
        for csv_name in ["body.csv", "joint.csv", "mesh.csv"]:
            Path(temp_hrdf_dir, csv_name).touch()
        
        mock_load_csv.return_value = []
        
        generator = UnifiedURDFGenerator(temp_hrdf_dir)
        generator.load()
        
        # Verify load_csv was called for existing files
        assert mock_load_csv.call_count == 3

    def test_generate_single_link_xml_basic(self, temp_hrdf_dir, mock_info_lists):
        """Test generating a single link XML element."""
        generator = UnifiedURDFGenerator(temp_hrdf_dir)
        generator.body_info_list = mock_info_lists["body"]
        generator.mesh_info_list = []
        generator.simple_geom_info_list = []
        
        parent_elem = ET.Element("robot")
        link_elem = generator.generate_single_link_xml(parent_elem, 0)
        
        assert link_elem.tag == "link"
        assert link_elem.get("name") == "base_link"
        
        # Check for inertial element
        inertial = link_elem.find("inertial")
        assert inertial is not None

    def test_generate_single_link_xml_with_prefix(self, temp_hrdf_dir, mock_info_lists):
        """Test generating a single link XML element with prefix."""
        generator = UnifiedURDFGenerator(temp_hrdf_dir)
        generator.body_info_list = mock_info_lists["body"]
        generator.mesh_info_list = []
        generator.simple_geom_info_list = []
        
        parent_elem = ET.Element("robot")
        link_elem = generator.generate_single_link_xml(parent_elem, 0, prefix="robot1")
        
        assert link_elem.get("name") == "robot1_base_link"

    def test_generate_single_link_xml_with_meshes(self, temp_hrdf_dir, mock_info_lists):
        """Test generating link XML with mesh geometries."""
        generator = UnifiedURDFGenerator(temp_hrdf_dir)
        generator.body_info_list = mock_info_lists["body"]
        generator.mesh_info_list = mock_info_lists["mesh"]
        generator.simple_geom_info_list = []
        
        parent_elem = ET.Element("robot")
        link_elem = generator.generate_single_link_xml(parent_elem, 0)
        
        # Check for visual and collision elements
        visual = link_elem.find("visual")
        collision = link_elem.find("collision")
        assert visual is not None
        assert collision is not None

    def test_add_visual_element_mesh(self, temp_hrdf_dir):
        """Test adding visual element with mesh geometry."""
        generator = UnifiedURDFGenerator(temp_hrdf_dir)
        
        link_elem = ET.Element("link")
        geom_info = Mock()
        geom_info.to_urdf_dict = Mock(side_effect=lambda tag: {
            "visual": {"name": "test_visual"},
            "origin": {"xyz": "0 0 0"},
            "geometry": {"filename": "test.obj"},
            "material": {"name": "test_material"}
        }.get(tag, {}))
        
        generator._add_visual_element(link_elem, geom_info, None, "visual_0")
        
        visual = link_elem.find("visual")
        assert visual is not None
        assert visual.get("name") == "visual_0"
        
        geometry = visual.find("geometry")
        mesh = geometry.find("mesh")
        assert mesh is not None
        assert mesh.get("filename") == "meshes/test.obj"

    def test_add_visual_element_simple_geometry(self, temp_hrdf_dir):
        """Test adding visual element with simple geometry."""
        generator = UnifiedURDFGenerator(temp_hrdf_dir)
        
        link_elem = ET.Element("link")
        geom_info = Mock()
        geom_info.to_urdf_dict = Mock(side_effect=lambda tag: {
            "visual": {"name": "test_visual"},
            "geometry": {"type": "box", "size": "1 1 1"}
        }.get(tag, {}))
        
        generator._add_visual_element(link_elem, geom_info, None, "visual_0")
        
        visual = link_elem.find("visual")
        geometry = visual.find("geometry")
        box = geometry.find("box")
        assert box is not None
        assert box.get("size") == "1 1 1"

    def test_generate_single_joint_xml_basic(self, temp_hrdf_dir, mock_info_lists):
        """Test generating a single joint XML element."""
        generator = UnifiedURDFGenerator(temp_hrdf_dir)
        generator.actuator_info_list = []
        
        parent_elem = ET.Element("robot")
        joint_info = mock_info_lists["joint"][0]
        
        joint_elem = generator.generate_single_joint_xml(
            parent_elem, "parent_link", "child_link", joint_info
        )
        
        assert joint_elem.tag == "joint"
        assert joint_elem.get("name") == "arm_joint"
        
        parent = joint_elem.find("parent")
        child = joint_elem.find("child")
        assert parent.get("link") == "parent_link"
        assert child.get("link") == "child_link"

    def test_generate_single_joint_xml_with_actuator(self, temp_hrdf_dir, mock_info_lists):
        """Test generating joint XML with actuator information."""
        generator = UnifiedURDFGenerator(temp_hrdf_dir)
        
        # Mock actuator info
        actuator_info = Mock()
        actuator_info.__getitem__ = Mock(side_effect=lambda key: Mock(data="arm_joint" if key == "joint_name" else ""))
        actuator_info.to_urdf_dict = Mock(return_value={"effort": "100", "velocity": "10"})
        generator.actuator_info_list = [actuator_info]
        
        parent_elem = ET.Element("robot")
        joint_info = mock_info_lists["joint"][0]
        
        joint_elem = generator.generate_single_joint_xml(
            parent_elem, "parent_link", "child_link", joint_info
        )
        
        limit = joint_elem.find("limit")
        assert limit is not None

    def test_generate_complete_urdf(self, temp_hrdf_dir, mock_info_lists):
        """Test generating complete URDF structure."""
        generator = UnifiedURDFGenerator(temp_hrdf_dir)
        generator.body_info_list = mock_info_lists["body"]
        generator.joint_info_list = mock_info_lists["joint"]
        generator.actuator_info_list = []
        generator.mesh_info_list = []
        generator.simple_geom_info_list = []
        generator.body_parent_id = [-1, 0]  # Second body is child of first
        
        generator.generate()
        
        # Check that links were generated
        links = generator.xml_root.findall("link")
        assert len(links) == 2
        
        # Check that joint was generated (only one since first body is root)
        joints = generator.xml_root.findall("joint")
        assert len(joints) == 1

    def test_generate_with_prefix(self, temp_hrdf_dir, mock_info_lists):
        """Test generating URDF with prefix."""
        generator = UnifiedURDFGenerator(temp_hrdf_dir)
        generator.body_info_list = mock_info_lists["body"]
        generator.joint_info_list = mock_info_lists["joint"]
        generator.actuator_info_list = []
        generator.mesh_info_list = []
        generator.simple_geom_info_list = []
        generator.body_parent_id = [-1, 0]
        
        generator.generate(prefix="test")
        
        links = generator.xml_root.findall("link")
        assert all("test_" in link.get("name") for link in links)

    @patch('hurodes.urdf_generator.unified_generator.find_info_by_attr')
    def test_generate_handles_missing_joint_info(self, mock_find_info, temp_hrdf_dir, mock_info_lists):
        """Test that generate handles missing joint information gracefully."""
        # Mock find_info_by_attr to return empty list for mesh/geom searches
        # but raise AssertionError for joint searches (when return_one=True)
        def mock_find_info_side_effect(attr_name, attr_value, info_list, return_one=False):
            if return_one and attr_name == "body_name" and attr_value in ["arm_link"]:
                # This is a joint search that should fail
                raise AssertionError("No joint found")
            else:
                # This is a mesh/geometry search that should return empty list
                return []
        
        mock_find_info.side_effect = mock_find_info_side_effect
        
        generator = UnifiedURDFGenerator(temp_hrdf_dir)
        generator.body_info_list = mock_info_lists["body"]
        generator.joint_info_list = []
        generator.actuator_info_list = []
        generator.mesh_info_list = []
        generator.simple_geom_info_list = []
        generator.body_parent_id = [-1, 0]
        
        # Should not raise an exception
        generator.generate()
        
        # Should still have links but no joints
        links = generator.xml_root.findall("link")
        joints = generator.xml_root.findall("joint")
        assert len(links) == 2
        assert len(joints) == 0 