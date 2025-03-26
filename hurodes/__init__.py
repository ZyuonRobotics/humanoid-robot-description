from os import path

PROJECT_PATH = path.dirname(path.dirname(__file__))
ASSETS_PATH = path.expanduser("~/.hurodes")
ROBOTS_PATH = path.join(ASSETS_PATH, "robots")
MJCF_ROBOTS_PATH = path.join(ASSETS_PATH, "mjcf_robots")
