from abc import ABC, abstractmethod

import xml.etree.ElementTree as ET


class MJCFGeneratorBase(ABC):
    def __init__(
            self,
            disable_gravity=False,
            timestep=0.001
    ):
        self.xml_root = ET.Element('mujoco')
        self.disable_gravity = disable_gravity
        self.time_step = timestep

        option = ET.SubElement(self.xml_root, 'option')
        if disable_gravity:
            ET.SubElement(option, 'flag', gravity="disable")
        if timestep:
            ET.SubElement(option, 'timestep', dt=str(timestep))

    @property
    def mjcf_str(self):
        tree = ET.ElementTree(self.xml_root)
        ET.indent(tree, space="  ", level=0)
        res = ET.tostring(self.xml_root, encoding='unicode', method='xml')
        return res

    @abstractmethod
    def load(self):
        raise NotImplemented("load method is not implemented")

    @abstractmethod
    def generate(self):
        raise NotImplemented("generate method is not implemented")

    def export(self, file_path=None):
        self.load()
        self.generate()
        if file_path is not None:
            with open(file_path, "w") as f:
                f.write(self.mjcf_str)
        return self.mjcf_str
