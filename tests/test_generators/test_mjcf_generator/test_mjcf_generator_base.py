import xml.etree.ElementTree as ET

from hurodes.generators.mjcf_generator.mjcf_generator_base import MJCFGeneratorBase
from hurodes.hrdf.hrdf import SimulatorConfig


class MJCFGeneratorBaseStub(MJCFGeneratorBase):
    def __init__(self):
        mock_simulator_config = SimulatorConfig()
        mock_simulator_config.timestep = 0.002
        mock_simulator_config.gravity = [0, 0, -9.81]
        mock_simulator_config.ground.type = "plane"
        mock_simulator_config.ground.contact_type = 1
        mock_simulator_config.ground.contact_affinity = 15
        mock_simulator_config.ground.friction = 1.0

        super().__init__(mock_simulator_config)
    
    def _load(self):
        pass

    def _clean(self):
        """Mock implementation of clean method"""
        # No specific cleanup needed for tests
        pass

    def _destroy(self):
        """Mock implementation of destroy method"""
        # No specific cleanup needed for tests
        pass

    def _generate(self):
        pass


def test_mjcf_generator_build():
    generator = MJCFGeneratorBaseStub()

    assert generator.xml_root is not None
    assert generator.xml_root.tag == 'mujoco'

    option_elem = generator.get_elem("option")
    assert option_elem is not None
    assert option_elem.tag == 'option'


def test_mjcf_generator_timestep_and_gravity():
    generator = MJCFGeneratorBaseStub()
    generator.load()
    generator.generate()
    option_elem = generator.get_elem("option")
    assert option_elem.get("timestep") == "0.002"
    assert option_elem.get("gravity") == "0 0 -9.81"


def test_add_scene_and_build():
    generator = MJCFGeneratorBaseStub()
    generator.add_scene()
    # Check if visual, asset, and worldbody elements are present
    assert generator.get_elem("visual") is not None
    assert generator.get_elem("asset") is not None
    assert generator.get_elem("worldbody") is not None

def test_all_body_names_and_body_tree_str():
    generator = MJCFGeneratorBaseStub()
    worldbody = generator.get_elem("worldbody")
    # Add a single body
    body = ET.SubElement(worldbody, "body", name="test_body")
    # Add a child body to test_body
    ET.SubElement(body, "body", name="child_body")
    # all_body_names should return both names
    names = generator.all_body_names
    assert "test_body" in names
    assert "child_body" in names
    # body_tree_str should contain both names
    tree_str = generator.body_tree_str
    assert "test_body" in tree_str
    assert "child_body" in tree_str
