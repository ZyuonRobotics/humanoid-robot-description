import xml.etree.ElementTree as ET

import mujoco

from hurodes.parsers.base_parser import BaseParser
from hurodes.utils.string import get_elem_tree_str


class HumanoidMJCFParser(BaseParser):

    def __init__(self, mjcf_path):
        super().__init__(mjcf_path)
        self.tree = ET.parse(self.file_path)
        self.root = self.tree.getroot()
        self.worldbody = self.root.find("worldbody")
        assert self.worldbody is not None, "No <worldbody> element found in the MJCF file."
        root_bodies = self.worldbody.findall("body")
        assert len(root_bodies) == 1, "There should be exactly one root <body> element in the <worldbody> element."
        self.base_link = root_bodies[0]

    def print_body_tree(self, colorful=False):
        print(get_elem_tree_str(self.base_link, colorful=colorful))

    @property
    def mujoco_spec(self):
        return mujoco.MjSpec.from_file(self.file_path) # type: ignore