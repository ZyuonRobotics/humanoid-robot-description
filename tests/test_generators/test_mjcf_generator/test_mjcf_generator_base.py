import xml.etree.ElementTree as ET
from unittest.mock import Mock
from hurodes.generators.mjcf_generator.mjcf_generator_base import MJCFGeneratorBase


class MJCFGeneratorBaseStub(MJCFGeneratorBase):
    def __init__(self):
        super().__init__()
        # Mock simulator_config for tests that need ground properties
        self._mock_simulator_config = Mock()
        self._mock_simulator_config.ground = Mock()
        self._mock_simulator_config.ground.type = "plane"
        self._mock_simulator_config.ground.contact_type = 1
        self._mock_simulator_config.ground.contact_affinity = 15
        self._mock_simulator_config.ground.friction = 1.0
    
    @property
    def simulator_config(self):
        return self._mock_simulator_config

    def load(self):
        pass

    def generate(self):
        pass


def test_mjcf_generator_build():
    generator = MJCFGeneratorBaseStub()

    assert generator.xml_root is not None
    assert generator.xml_root.tag == 'mujoco'

    option_elem = generator.get_elem("option")
    assert option_elem is not None
    assert option_elem.tag == 'option'


def test_mjcf_generator_timestep():
    generator = MJCFGeneratorBaseStub()
    # The timestep is no longer set in constructor, but we can test 
    # that we can manually set it in the option element
    option_elem = generator.get_elem("option")
    option_elem.set("timestep", "0.1")
    assert option_elem.get("timestep") == "0.1"


def test_add_scene_and_build():
    generator = MJCFGeneratorBaseStub()
    generator.add_scene()
    # Check if visual, asset, and worldbody elements are present
    assert generator.get_elem("visual") is not None
    assert generator.get_elem("asset") is not None
    assert generator.get_elem("worldbody") is not None


def test_export_returns_string(tmp_path):
    generator = MJCFGeneratorBaseStub()
    xml_str = generator.export()
    assert isinstance(xml_str, str)
    # Test export to file
    file_path = tmp_path / "test.xml"
    xml_str2 = generator.export(file_path)
    assert file_path.exists()
    assert xml_str2 == file_path.read_text()


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
