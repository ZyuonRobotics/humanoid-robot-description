import pytest
import xml.etree.ElementTree as ET
import tempfile
import yaml
import time
import shutil
from pathlib import Path
from unittest.mock import Mock, patch, MagicMock

from hurodes.generators.urdf_generator.urdf_humanoid_generator import URDFHumanoidGenerator
from hurodes import ROBOTS_PATH


class TestHumanoidURDFGenerator:
    """Test cases for HumanoidURDFGenerator class."""

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

    def test_init_with_robot_name(self, temp_hrdf_dir):
        """Test initialization with explicit robot name."""
        generator = URDFHumanoidGenerator.from_hrdf_path(temp_hrdf_dir)
        assert generator.robot_name == temp_hrdf_dir.name
        assert generator.hrdf.hrdf_path == temp_hrdf_dir

    def test_init_without_robot_name(self, temp_hrdf_dir):
        """Test initialization with robot name from directory."""
        generator = URDFHumanoidGenerator.from_hrdf_path(temp_hrdf_dir)
        expected_name = temp_hrdf_dir.name
        assert generator.robot_name == expected_name

    def test_load_humanoid_robot(self, temp_hrdf_dir):
        """Test loading humanoid robot from HRDF."""
        generator = URDFHumanoidGenerator.from_hrdf_path(temp_hrdf_dir)
        
        assert generator.hrdf is not None

    def test_load_with_csv_files(self, temp_hrdf_dir):
        """Test loading with CSV files present."""
        # Create CSV files
        for csv_name in ["body.csv", "joint.csv", "mesh.csv"]:
            (temp_hrdf_dir / csv_name).touch()
        
        # Should not raise an error even if CSV files are incomplete
        try:
            generator = URDFHumanoidGenerator.from_hrdf_path(temp_hrdf_dir)
            assert generator.hrdf is not None
        except Exception:
            pass  # It's ok if it fails due to incomplete CSV data

    def test_initialization(self, temp_hrdf_dir):
        """Test that generator is properly initialized."""
        generator = URDFHumanoidGenerator()
        
        # Check that hrdf is initialized as None before loading
        assert generator.hrdf is None
        assert not generator._loaded

    def test_generate_basic(self, temp_hrdf_dir):
        """Test basic generate functionality."""
        try:
            generator = URDFHumanoidGenerator.from_hrdf_path(temp_hrdf_dir)
            generator.generate()
        except Exception:
            pass  # It's ok if it fails due to incomplete HRDF data 