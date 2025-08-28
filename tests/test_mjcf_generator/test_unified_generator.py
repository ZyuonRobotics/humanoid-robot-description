import pytest
import tempfile
import json
from pathlib import Path
from unittest.mock import Mock, patch

from hurodes.mjcf_generator.unified_generator import (
    UnifiedMJCFGenerator, 
    get_prefix_name, 
    find_info_by_attr
)
from hurodes import ROBOTS_PATH


class TestMJCFHelperFunctions:
    """Test cases for MJCF helper functions."""

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


class TestUnifiedMJCFGenerator:
    """Test cases for UnifiedMJCFGenerator class."""

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
            
            yield temp_dir

    def test_unified_mjcf_generator_init_default_params(self, temp_hrdf_dir):
        """Test initialization with default parameters."""
        generator = UnifiedMJCFGenerator(hrdf_path=temp_hrdf_dir)
        assert generator.hrdf_path == temp_hrdf_dir
        assert generator.disable_gravity is False
        assert generator.time_step == 0.001

    def test_unified_mjcf_generator_init_custom_params(self, temp_hrdf_dir):
        """Test initialization with custom parameters."""
        generator = UnifiedMJCFGenerator(
            hrdf_path=temp_hrdf_dir,
            disable_gravity=True,
            timestep=0.005
        )
        assert generator.hrdf_path == temp_hrdf_dir
        assert generator.disable_gravity is True
        assert generator.time_step == 0.005

    def test_load_meta_json(self, temp_hrdf_dir):
        """Test loading meta.json file."""
        generator = UnifiedMJCFGenerator(hrdf_path=temp_hrdf_dir)
        generator.load()
        
        assert generator.body_parent_id == [-1, 0, 0]
        assert generator.mesh_file_type == "obj"
        assert generator.ground_dict["name"] == "ground"

    @patch('hurodes.mjcf_generator.unified_generator.load_csv')
    def test_load_csv_files(self, mock_load_csv, temp_hrdf_dir):
        """Test loading CSV files."""
        # Create CSV files
        for csv_name in ["body.csv", "joint.csv", "mesh.csv"]:
            Path(temp_hrdf_dir, csv_name).touch()
        
        mock_load_csv.return_value = []
        
        generator = UnifiedMJCFGenerator(hrdf_path=temp_hrdf_dir)
        generator.load()
        
        # Verify load_csv was called for existing files
        assert mock_load_csv.call_count == 3

    def test_info_lists_initialization(self, temp_hrdf_dir):
        """Test that info lists are properly initialized."""
        generator = UnifiedMJCFGenerator(hrdf_path=temp_hrdf_dir)
        
        # Check that all info lists are initialized as empty lists
        assert isinstance(generator.body_info_list, list)
        assert isinstance(generator.joint_info_list, list)
        assert isinstance(generator.actuator_info_list, list)
        assert isinstance(generator.mesh_info_list, list)
        assert isinstance(generator.simple_geom_info_list, list)
        assert isinstance(generator.body_parent_id, list)
        assert isinstance(generator.ground_dict, dict)

    def test_mesh_file_type_initialization(self, temp_hrdf_dir):
        """Test mesh file type initialization."""
        generator = UnifiedMJCFGenerator(hrdf_path=temp_hrdf_dir)
        assert generator.mesh_file_type is None
        
        generator.load()
        assert generator.mesh_file_type == "obj"
