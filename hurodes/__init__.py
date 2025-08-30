from pathlib import Path
import os

PROJECT_PATH = Path(__file__).resolve().parent.parent

# Get ASSETS_PATH from environment variable HURODES_ASSETS_PATH, fallback to default if not set
ASSETS_PATH = Path(os.getenv("HURODES_ASSETS_PATH", "~/.hurodes")).expanduser()
ROBOTS_PATH = ASSETS_PATH / "robots"
MJCF_ROBOTS_PATH = ASSETS_PATH / "mjcf_robots"
URDF_ROBOTS_PATH = ASSETS_PATH / "urdf_robots"
