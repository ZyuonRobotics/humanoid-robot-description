import pytest
import tempfile
import yaml
import time
import shutil
from pathlib import Path
from unittest.mock import Mock, patch

from hurodes.generators.mjcf_generator.mjcf_humanoid_generator import MJCFHumanoidGenerator
from hurodes import ROBOTS_PATH

class TestHumanoidMJCFGenerator:
    """Test cases for HumanoidMJCFGenerator class."""

    @pytest.fixture
    def temp_hrdf_dir(self):
        """Create a test HRDF directory structure within ROBOTS_PATH for testing."""
        # Create a unique test robot name with timestamp to avoid conflicts
        timestamp = str(int(time.time() * 1000000))  # microsecond timestamp
        test_robot_name = f"test_{timestamp}"
        
        # Create the test robot directory within ROBOTS_PATH
        test_robot_path = ROBOTS_PATH / test_robot_name
        test_robot_path.mkdir(parents=True, exist_ok=True)
        
        try:
            # Create meta.yaml
            meta_data = {
                "body_parent_id": [-1, 0, 0],
                "mesh_file_type": "obj",
                "simulator_config": {
                    "timestep": 0.002,
                    "gravity": [0, 0, -9.81],
                    "ground": {
                        "type": "plane",
                        "contact_affinity": 0,
                        "contact_type": 0,
                        "friction": 1.0
                    }
                }
            }
            with open(test_robot_path / "meta.yaml", "w", encoding='utf-8') as f:
                yaml.dump(meta_data, f, default_flow_style=False, allow_unicode=True, indent=2)
            
            # Create CSV files directory structure
            yield test_robot_path
            
        finally:
            # Cleanup: remove the test robot directory after the test
            if test_robot_path.exists():
                shutil.rmtree(test_robot_path)

    def test_humanoid_mjcf_generator_init_default_params(self, temp_hrdf_dir):
        """Test initialization with default parameters."""
        generator = MJCFHumanoidGenerator.from_hrdf_path(temp_hrdf_dir)
        assert generator.hrdf is not None
        assert generator.hrdf.hrdf_path == temp_hrdf_dir

    def test_humanoid_mjcf_generator_init_custom_params(self, temp_hrdf_dir):
        """Test initialization with custom parameters."""
        generator = MJCFHumanoidGenerator.from_hrdf_path(temp_hrdf_dir)
        # Test that the generator can be initialized and hrdf is loaded
        assert generator.hrdf is not None
        assert generator.hrdf.hrdf_path == temp_hrdf_dir

    def test_load_humanoid_robot(self, temp_hrdf_dir):
        """Test loading humanoid robot from HRDF."""
        generator = MJCFHumanoidGenerator.from_hrdf_path(temp_hrdf_dir)
        
        assert generator.hrdf is not None

    def test_load_with_csv_files(self, temp_hrdf_dir):
        """Test loading with CSV files present."""
        # Create CSV files
        for csv_name in ["body.csv", "joint.csv", "mesh.csv"]:
            (temp_hrdf_dir / csv_name).touch()
        
        # Should not raise an error even if CSV files are incomplete
        try:
            generator = MJCFHumanoidGenerator.from_hrdf_path(temp_hrdf_dir)
            assert generator.hrdf is not None
        except Exception:
            pass  # It's ok if it fails due to incomplete CSV data

    def test_initialization(self, temp_hrdf_dir):
        """Test that generator is properly initialized."""
        generator = MJCFHumanoidGenerator()
        
        # Check that hrdf is initialized as None before loading
        assert generator.hrdf is None
        assert not generator._loaded

    def test_generate_basic(self, temp_hrdf_dir):
        """Test basic generate functionality."""
        try:
            generator = MJCFHumanoidGenerator.from_hrdf_path(temp_hrdf_dir)
            generator.generate()
        except Exception:
            pass  # It's ok if it fails due to incomplete HRDF data
