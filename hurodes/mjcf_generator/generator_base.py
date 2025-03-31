from abc import ABC, abstractmethod

import xml.etree.ElementTree as ET

from hurodes.mjcf_generator.constants import *

class MJCFGeneratorBase(ABC):
    def __init__(
            self,
            disable_gravity=False,
            timestep=0.001
    ):
        self.xml_root = ET.Element('mujoco')
        self.disable_gravity = disable_gravity
        self.time_step = timestep
        self.ground_dict = None

        if disable_gravity:
            ET.SubElement(self.get_elem("option"), 'flag', gravity="disable")
        if timestep:
            self.get_elem("option").set('timestep', str(timestep))

    def get_elem(self, elem_name):
        elem_num = len(self.xml_root.findall(elem_name))
        assert elem_num <= 1, f"Multiple {elem_name} elements found"
        if elem_num == 1:
            return self.xml_root.find(elem_name)
        else:
            return ET.SubElement(self.xml_root, elem_name)

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

    def add_scene(self):
        # visual
        visual_elem = self.get_elem("visual")
        headlight_elem = ET.SubElement(visual_elem, 'headlight',
                                       attrib={"diffuse": "0.6 0.6 0.6", "ambient": "0.3 0.3 0.3", "specular": "0 0 0"})
        rgba_elem = ET.SubElement(visual_elem, 'rgba', attrib={"haze": "0.15 0.25 0.35 1"})
        global_elem = ET.SubElement(visual_elem, 'global', attrib={"azimuth": "160", "elevation": "-20"})

        # asset
        asset_elem = self.get_elem("asset")
        ET.SubElement(asset_elem, "texture", attrib=DEFAULT_SKY_TEXTURE_ATTR)
        ET.SubElement(asset_elem, "texture", attrib=DEFAULT_GROUND_TEXTURE_ATTR)
        ET.SubElement(asset_elem, "material", attrib=DEFAULT_GROUND_MATERIAL_ATTR)

        # ground
        worldbody_elem = self.get_elem("worldbody")
        light_elem = ET.SubElement(worldbody_elem, 'light', attrib=DEFAULT_SKY_LIGHT_ATTR)
        ground_attr = DEFAULT_GROUND_GEOM_ATTR | (self.ground_dict or {})
        geom_elem = ET.SubElement(self.get_elem("worldbody"), 'geom', attrib=ground_attr)

    def build(self):
        self.load()
        self.generate()
        self.add_scene()

    def export(self, file_path=None):
        self.build()
        if file_path is not None:
            with open(file_path, "w") as f:
                f.write(self.mjcf_str)
        return self.mjcf_str

class MJCFGeneratorComposite(MJCFGeneratorBase):
    def __init__(self, generators, **kwargs):
        super().__init__(**kwargs)
        self.generators = generators

    def load(self):
        for generator in self.generators:
            generator.load()

    def generate(self):
        for generator in self.generators:
            generator.generate()
            for top_elem in generator.xml_root:
                new_top_elem = self.get_elem(top_elem.tag)
                new_top_elem.attrib |= top_elem.attrib
                for elem in top_elem:
                    new_top_elem.append(elem)
