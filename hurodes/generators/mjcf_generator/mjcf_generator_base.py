import xml.etree.ElementTree as ET

from hurodes.generators.generator_base import GeneratorBase
from hurodes.generators.mjcf_generator.scene_constants import (
    DEFAULT_VISUAL_ELEM, 
    DEFAULT_TEXTURE_ELEM, 
    DEFAULT_MATERIAL_ATTR, 
    DEFAULT_LIGHT_ATTR,
    DEFAULT_GROUND_GEOM_ATTR
)
from hurodes.utils.string import get_elem_tree_str
from hurodes.hrdf.hrdf import SimulatorConfig

class MJCFGeneratorBase(GeneratorBase):
    """
    Base class for MJCF (MuJoCo XML Format) generators.
    """
    
    def __init__(self, simulator_config:SimulatorConfig = None):
        super().__init__()

        self.simulator_config = simulator_config

    def _xml_root_init(self):
        """
        Initialize the XML root element.
        """
        self._xml_root = ET.Element('mujoco')
        option_elem = ET.SubElement(self._xml_root, 'option')
        if self.simulator_config is not None:
            option_elem.set("timestep", str(self.simulator_config.timestep))
            option_elem.set("gravity", " ".join(map(str, self.simulator_config.gravity)))

    def _clean(self):
        """
        Clean up MJCF-specific data.
        """
        # Reset simulator config if needed
        pass

    def add_scene(self):
        """Add visual scene elements including lighting, textures, and ground plane."""
        # visual
        visual_elem = self.get_elem("visual")
        ET.SubElement(visual_elem, 'headlight', attrib=DEFAULT_VISUAL_ELEM["headlight"])
        ET.SubElement(visual_elem, 'rgba', attrib=DEFAULT_VISUAL_ELEM["rgba"])
        ET.SubElement(visual_elem, 'global', attrib=DEFAULT_VISUAL_ELEM["global"])

        # asset
        asset_elem = self.get_elem("asset")
        ET.SubElement(asset_elem, "texture", attrib=DEFAULT_TEXTURE_ELEM["skybox"])
        ET.SubElement(asset_elem, "texture", attrib=DEFAULT_TEXTURE_ELEM["plane"])
        ET.SubElement(asset_elem, "material", attrib=DEFAULT_MATERIAL_ATTR["plane"])

        # light
        ET.SubElement(self.get_elem("worldbody"), 'light', attrib=DEFAULT_LIGHT_ATTR)

        # ground
        ground_attr = DEFAULT_GROUND_GEOM_ATTR
        if self.simulator_config is not None:
            ground_attr.update({
                "type": str(self.simulator_config.ground.type),
                "contype": str(self.simulator_config.ground.contact_type),
                "conaffinity": str(self.simulator_config.ground.contact_affinity),
                "friction": f"{self.simulator_config.ground.friction} 0.005 0.0001",
            })
        ET.SubElement(self.get_elem("worldbody"), 'geom', attrib=ground_attr)

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
