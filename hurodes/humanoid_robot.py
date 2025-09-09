from pathlib import Path
from typing import Optional, List
import os

from hurodes.hrdf.hrdf import HRDF
from hurodes.generators.mjcf_generator.mjcf_humanoid_generator import MJCFHumanoidGenerator
from hurodes.generators.urdf_generator.urdf_humanoid_generator import URDFHumanoidGenerator
from hurodes import ROBOTS_PATH

def is_robot_path(path: Path) -> bool:
    return path.is_dir() and (path / "meta.yaml").exists()

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
            
        for robot_dir in ROBOTS_PATH.iterdir():
            if is_robot_path(robot_dir):
                robot_names.append(robot_dir.relative_to(ROBOTS_PATH).as_posix())
        
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
        assert is_robot_path(robot_path), f"Robot directory not found: {robot_path}"
        
        return cls(HRDF.from_dir(robot_path))
    
    def export_mjcf(self, output_path: Path, **kwargs) -> str:
        generator = MJCFHumanoidGenerator.from_hrdf(self.hrdf)
        return generator.export(output_path, **kwargs)
    
    def export_urdf(self, output_path: Path, **kwargs) -> str:
        generator = URDFHumanoidGenerator.from_hrdf(self.hrdf)
        return generator.export(output_path, **kwargs)        

    def __getattr__(self, name: str):
        return getattr(self.hrdf, name)
    
    def __repr__(self) -> str:
        """Return string representation of the HumanoidRobot."""
        return f"HumanoidRobot(name='{self.robot_name}')"
    
    def __str__(self) -> str:
        """Return human-readable string representation."""
        return f"Humanoid Robot: {self.robot_name}" 