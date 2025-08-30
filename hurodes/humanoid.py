from pathlib import Path
from typing import List, Optional

from hurodes import ROBOTS_PATH
from hurodes.hrdf.hrdf import HumanoidRobot


class Asset:
    def __init__(self, name: str, description: str, path: Path):
        self.name = name
        self.description = description
        self.path = Path(path)

    def __str__(self):
        return f"{self.name}: {self.description}"
    
    def __repr__(self):
        return f"Asset(name={self.name}, description={self.description}, path={self.path})"