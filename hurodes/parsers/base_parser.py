from pathlib import Path
from abc import ABC, abstractmethod

from hurodes.hrdf.hrdf import HRDF


class BaseParser(ABC):
    def __init__(self, file_path, robot_name):
        self.file_path = file_path
        self.robot_name = robot_name
        self.hrdf = HRDF()

        self.mesh_path = {}
        self.simulator_dict = {}

    @abstractmethod
    def parse(self, base_link_name="base_link"):
        """Parse the robot file and populate the HRDF structure"""
        pass

    def save(self, max_faces=40000):
        """Save the parsed robot data using HRDF's save method"""
        self.hrdf.save(mesh_path=self.mesh_path, max_faces=max_faces)

    @abstractmethod
    def print_body_tree(self, colorful=False):
        """Print the body tree structure"""
        pass
