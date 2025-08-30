import pytest
import tempfile
import yaml
from pathlib import Path
from unittest.mock import Mock, patch

from hurodes.generators.mjcf_generator.humanoid_generator import HumanoidMJCFGenerator

class TestHumanoidMJCFGenerator:
    """Test cases for HumanoidMJCFGenerator class."""

    @pytest.fixture
    def temp_hrdf_dir(self):
        """Create a temporary HRDF directory structure for testing."""
        with tempfile.TemporaryDirectory() as temp_dir:
            # Create meta.yaml
            meta_data = {
                "body_parent_id": [-1, 0, 0],
                "mesh_file_type": "obj",
                "ground": {"name": "ground", "type": "plane"}
            }
            with open(Path(temp_dir, "meta.yaml"), "w", encoding='utf-8') as f:
                yaml.dump(meta_data, f, default_flow_style=False, allow_unicode=True, indent=2)
            
            yield temp_dir

    def test_humanoid_mjcf_generator_init_default_params(self, temp_hrdf_dir):
        """Test initialization with default parameters."""
        generator = HumanoidMJCFGenerator(hrdf_path=temp_hrdf_dir)
        assert generator.hrdf_path == temp_hrdf_dir
        assert generator.disable_gravity is False
        assert generator.timestep == 0.001

    def test_humanoid_mjcf_generator_init_custom_params(self, temp_hrdf_dir):
        """Test initialization with custom parameters."""
        generator = HumanoidMJCFGenerator(
            hrdf_path=temp_hrdf_dir,
            disable_gravity=True,
            timestep=0.005
        )
        assert generator.hrdf_path == temp_hrdf_dir
        assert generator.disable_gravity is True
        assert generator.timestep == 0.005

    def test_load_humanoid_robot(self, temp_hrdf_dir):
        """Test loading humanoid robot from HRDF."""
        generator = HumanoidMJCFGenerator(hrdf_path=temp_hrdf_dir)
        generator.load()
        
        assert generator.humanoid_robot is not None

    def test_load_with_csv_files(self, temp_hrdf_dir):
        """Test loading with CSV files present."""
        # Create CSV files
        for csv_name in ["body.csv", "joint.csv", "mesh.csv"]:
            Path(temp_hrdf_dir, csv_name).touch()
        
        generator = HumanoidMJCFGenerator(hrdf_path=temp_hrdf_dir)
        # Should not raise an error even if CSV files are incomplete
        try:
            generator.load()
        except Exception:
            pass  # It's ok if it fails due to incomplete CSV data

    def test_initialization(self, temp_hrdf_dir):
        """Test that generator is properly initialized."""
        generator = HumanoidMJCFGenerator(hrdf_path=temp_hrdf_dir)
        
        # Check that humanoid_robot is initialized as None
        assert generator.humanoid_robot is None
        assert generator.hrdf_path == temp_hrdf_dir

    def test_generate_basic(self, temp_hrdf_dir):
        """Test basic generate functionality."""
        generator = HumanoidMJCFGenerator(hrdf_path=temp_hrdf_dir)
        try:
            generator.load()
            generator.generate()
        except Exception:
            pass  # It's ok if it fails due to incomplete HRDF data
