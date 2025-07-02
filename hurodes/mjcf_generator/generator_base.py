from abc import ABC, abstractmethod
from typing import List, Dict, Union
import xml.etree.ElementTree as ET

from hurodes.mjcf_generator.constants import *
from hurodes.utils import get_elem_tree_str

class MJCFGeneratorBase(ABC):
    def __init__(
            self,
            disable_gravity=False,
            timestep=0.001
    ):
        self.disable_gravity = disable_gravity
        self.time_step = timestep

        self.xml_root: ET.Element | None = None
        self.ground_dict: dict | None = None

    def init_xml_root(self):
        self.xml_root = ET.Element('mujoco')
        if self.disable_gravity:
            ET.SubElement(self.get_elem("option"), 'flag', gravity="disable")
        if self.time_step:
            self.get_elem("option").set('timestep', str(self.time_step))

    def get_elem(self, elem_name) -> ET.Element:
        assert self.xml_root is not None, "xml_root is not initialized"
        elems = self.xml_root.findall(elem_name)
        assert len(elems) <= 1, f"Multiple {elem_name} elements found"
        if len(elems) == 1:
            return elems[0]
        else:
            return ET.SubElement(self.xml_root, elem_name)

    @property
    def mjcf_str(self) -> str:
        tree = ET.ElementTree(self.xml_root)
        ET.indent(tree, space="  ", level=0)
        assert self.xml_root is not None, "xml_root is not initialized"
        res = ET.tostring(self.xml_root, encoding='unicode', method='xml')
        return res

    @abstractmethod
    def load(self):
        raise NotImplementedError("load method is not implemented")

    @abstractmethod
    def generate(self, prefix=None):
        raise NotImplementedError("generate method is not implemented")

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
        self.init_xml_root()
        self.load()
        self.generate()
        self.add_scene()

    def export(self, file_path=None):
        self.build()
        if file_path is not None:
            with open(file_path, "w") as f:
                f.write(self.mjcf_str)
        return self.mjcf_str

    @property
    def all_body_names(self):
        assert self.xml_root is not None, "xml_root is not initialized"
        body_list = [elem.get("name") for elem in self.xml_root.findall(".//body")]
        assert None not in body_list, "None body name found"
        return body_list

    @property
    def body_tree_str(self):
        worldbody_elem = self.get_elem("worldbody")
        body_elems = worldbody_elem.findall("body")
        assert len(body_elems) == 1, "Multiple body elements found"
        return get_elem_tree_str(body_elems[0], colorful=False)
