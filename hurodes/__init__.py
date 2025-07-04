from pathlib import Path

PROJECT_PATH = Path(__file__).resolve().parent.parent
ASSETS_PATH = Path("~/.hurodes").expanduser()
ROBOTS_PATH = ASSETS_PATH / "robots"
MJCF_ROBOTS_PATH = ASSETS_PATH / "mjcf_robots"
