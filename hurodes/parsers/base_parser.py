from pathlib import Path
from abc import ABC, abstractmethod

from hurodes.hrdf.hrdf import HRDF
from hurodes.utils.string import filter_str_list

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

    def parse_body_name(self):
        body_names = self.hrdf.info_list_dict["body"].get_data_list("name")

        for name in ["hip", "knee", "foot"]:
            left_body_names = filter_str_list(body_names, pos_strings=["left", name])
            right_body_names = filter_str_list(body_names, pos_strings=["right", name])
            if len(left_body_names) == len(right_body_names) == 1:
                names = [left_body_names[0], right_body_names[0]]
                setattr(self.hrdf.body_name_config, f"{name}_names", names)

        torso_body_names = filter_str_list(body_names, pos_strings=["torso"])
        if len(torso_body_names) == 1:
            self.hrdf.body_name_config.torso_name = torso_body_names[0]
