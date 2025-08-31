from pathlib import Path
from typing import Optional, List
import os

from hurodes.hrdf.hrdf import HRDF
from hurodes.generators.mjcf_generator.mjcf_humanoid_generator import MJCFHumanoidGenerator
from hurodes.generators.urdf_generator.urdf_humanoid_generator import URDFHumanoidGenerator
from hurodes import ROBOTS_PATH


class HumanoidRobot:
    """
    HumanoidRobot class for managing humanoid robot descriptions and generation.
    
    This class provides a high-level interface for working with humanoid robots,
    including loading HRDF descriptions, listing available robots, and generating
    MJCF and URDF files.
    """
    
    def __init__(self, hrdf: HRDF):
        """
        Initialize HumanoidRobot with an HRDF object.
        
        Args:
            hrdf: HRDF object containing robot description
        """
        assert isinstance(hrdf, HRDF), "hrdf must be an instance of HRDF"
        
        self.hrdf = hrdf
        self.robot_name = hrdf.robot_name
    
    @classmethod
    def list_robots(cls) -> List[str]:
        """
        List all available robot names from the robots directory.
        
        Returns:
            List of robot names (directory names in the robots path)
        """
        robot_names = []
        
        if not ROBOTS_PATH.exists():
            return robot_names
            
        # Iterate through all directories in ROBOTS_PATH
        for robot_dir in ROBOTS_PATH.iterdir():
            if robot_dir.is_dir():
                # Check if it's a valid HRDF directory by looking for meta.yaml
                meta_file = robot_dir / "meta.yaml"
                if meta_file.exists():
                    # Use relative path from ROBOTS_PATH as robot name
                    robot_name = robot_dir.relative_to(ROBOTS_PATH).as_posix()
                    robot_names.append(robot_name)
        
        return sorted(robot_names)
    
    @classmethod
    def from_name(cls, robot_name: str) -> 'HumanoidRobot':
        """
        Create a HumanoidRobot instance from a robot name.
        
        Args:
            robot_name: Name of the robot (relative path from robots directory)
            
        Returns:
            HumanoidRobot instance
            
        Raises:
            FileNotFoundError: If the robot directory or meta.yaml doesn't exist
            ValueError: If the robot name is invalid
        """
        if not robot_name:
            raise ValueError("Robot name cannot be empty")
        
        robot_path = ROBOTS_PATH / robot_name
        assert robot_path.exists(), f"Robot directory not found: {robot_path}"
        assert robot_path.is_dir(), f"Robot path is not a directory: {robot_path}"
        
        meta_file = robot_path / "meta.yaml"
        assert meta_file.exists(), f"meta.yaml not found in robot directory: {robot_path}"
        
        # Load HRDF from directory
        hrdf = HRDF.from_dir(robot_path)
        
        return cls(hrdf)
    
    def generate_mjcf(self, output_path: Optional[Path] = None) -> str:
        """
        Generate MJCF file from the robot's HRDF description.
        
        Args:
            output_path: Optional path to save the MJCF file. If None, 
                        saves to default location in MJCF_ROBOTS_PATH
                        
        Returns:
            String content of the generated MJCF file
            
        Raises:
            RuntimeError: If MJCF generation fails
        """
        # Create MJCF generator from HRDF
        generator = MJCFHumanoidGenerator.from_hrdf(self.hrdf)
        
        # Generate the MJCF content
        generator.generate()
        mjcf_content = generator.xml_str
        
        # Determine output path
        if output_path is None:
            from hurodes import MJCF_ROBOTS_PATH
            output_path = MJCF_ROBOTS_PATH / f"{self.robot_name}.xml"
        
        # Ensure output directory exists
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        # Save to file
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(mjcf_content)
        
        return mjcf_content
    
    def generate_urdf(self, output_path: Optional[Path] = None) -> str:
        """
        Generate URDF file from the robot's HRDF description.
        
        Args:
            output_path: Optional path to save the URDF file. If None,
                        saves to default location in URDF_ROBOTS_PATH
                        
        Returns:
            String content of the generated URDF file
            
        Raises:
            RuntimeError: If URDF generation fails
        """
        # Create URDF generator from HRDF
        generator = URDFHumanoidGenerator.from_hrdf(self.hrdf)
        
        # Generate the URDF content
        generator.generate()
        urdf_content = generator.xml_str
        
        # Determine output path
        if output_path is None:
            from hurodes import URDF_ROBOTS_PATH
            output_path = URDF_ROBOTS_PATH / f"{self.robot_name}.urdf"
        
        # Ensure output directory exists
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        # Save to file
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(urdf_content)
        
        return urdf_content
    
    def __repr__(self) -> str:
        """Return string representation of the HumanoidRobot."""
        return f"HumanoidRobot(name='{self.robot_name}')"
    
    def __str__(self) -> str:
        """Return human-readable string representation."""
        return f"Humanoid Robot: {self.robot_name}" 