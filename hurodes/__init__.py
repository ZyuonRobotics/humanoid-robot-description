from pathlib import Path

# never used PROJECT_PATH
# PROJECT_PATH = path.dirname(path.dirname(__file__))
# ASSETS_PATH = path.expanduser("~/.hurodes")
# ROBOTS_PATH = path.join(ASSETS_PATH, "robots")
# MJCF_ROBOTS_PATH = path.join(ASSETS_PATH, "mjcf_robots")

PROJECT_PATH = Path(__file__).resolve().parent.parent
ASSETS_PATH = Path("~/.hurodes").expanduser()
ROBOTS_PATH = ASSETS_PATH / "robots"
MJCF_ROBOTS_PATH = ASSETS_PATH / "mjcf_robots"
