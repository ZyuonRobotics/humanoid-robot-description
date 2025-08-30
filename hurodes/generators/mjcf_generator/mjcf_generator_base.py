from abc import ABC, abstractmethod
import xml.etree.ElementTree as ET

from hurodes.generators.generator_base import GeneratorBase
from hurodes.generators.mjcf_generator.constants import *
from hurodes.utils.string import get_elem_tree_str

class MJCFGeneratorBase(GeneratorBase):
    """
    Base class for MJCF (MuJoCo XML Format) generators.
    
    This class extends GeneratorBase with MJCF-specific functionality
    including gravity settings, timestep configuration, and scene setup.
    """
    
    def __init__(self):
        super().__init__()

    @property
    def xml_root(self) -> ET.Element:
        """Get or create the MJCF root element with MJCF-specific configuration."""
        if self._xml_root is None:
            self._xml_root = ET.Element('mujoco')
        return self._xml_root

    def add_scene(self):
        """Add visual scene elements including lighting, textures, and ground plane."""
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
        ground_attr = DEFAULT_GROUND_GEOM_ATTR
        ground_attr.update({
            "type": str(self.simulator_config.ground.type),
            "contype": str(self.simulator_config.ground.contact_type),
            "conaffinity": str(self.simulator_config.ground.contact_affinity),
            "friction": f"{self.simulator_config.ground.friction} 0.005 0.0001",
        })
        geom_elem = ET.SubElement(self.get_elem("worldbody"), 'geom', attrib=ground_attr)

    def build(self):
        """Build the complete MJCF including scene elements."""
        super().build()  # Call parent build method
        self.add_scene()    # Add MJCF-specific scene elements

    @property
    def all_body_names(self):
        """Get all body names in the MJCF."""
        body_list = [elem.get("name") for elem in self.xml_root.findall(".//body")]
        assert None not in body_list, "None body name found"
        return body_list

    @property
    def body_tree_str(self):
        """Get a string representation of the body tree structure."""
        worldbody_elem = self.get_elem("worldbody")
        body_elems = worldbody_elem.findall("body")
        assert len(body_elems) == 1, "Multiple body elements found"
        return get_elem_tree_str(body_elems[0], colorful=False)
