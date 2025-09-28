from pathlib import Path
import os

from hurodes.humanoid_robot import HumanoidRobot

PROJECT_PATH = Path(__file__).resolve().parent.parent

# Get ASSETS_PATH from environment variable HURODES_ASSETS_PATH, fallback to default if not set
ASSETS_PATH = Path(os.getenv("HURODES_ASSETS_PATH", "~/.hurodes")).expanduser()
ROBOTS_PATH = ASSETS_PATH / "robots"

VERSION = "0.1.0"


__all__ = ["HumanoidRobot"]