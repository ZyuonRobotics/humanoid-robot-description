import xml.etree.ElementTree as ET
from hurodes.mjcf_generator.generator_base import MJCFGeneratorBase


class TestMJCFGeneratorBase(MJCFGeneratorBase):
    def load(self):
        pass

    def generate(self):
        pass


def test_mjcf_generator_build():
    generator = TestMJCFGeneratorBase()

    generator.init_xml_root()
    assert generator.xml_root is not None
    assert generator.xml_root.tag == 'mujoco'

    option_elem = generator.get_elem("option")
    assert option_elem is not None
    assert option_elem.tag == 'option'

def test_mjcf_generator_timestep():
    generator = TestMJCFGeneratorBase(timestep=0.1)
    generator.init_xml_root()
    assert generator.get_elem("option").get("timestep") == "0.1"

def test_add_scene_and_build():
    generator = TestMJCFGeneratorBase()
    generator.ground_dict = {"size": "1 1 0.1"}
    generator.init_xml_root()
    generator.add_scene()
    # Check if visual, asset, and worldbody elements are present
    assert generator.get_elem("visual") is not None
    assert generator.get_elem("asset") is not None
    assert generator.get_elem("worldbody") is not None

def test_export_returns_string(tmp_path):
    generator = TestMJCFGeneratorBase()
    xml_str = generator.export()
    assert isinstance(xml_str, str)
    # Test export to file
    file_path = tmp_path / "test.xml"
    xml_str2 = generator.export(str(file_path))
    assert file_path.exists()
    assert xml_str2 == file_path.read_text()

def test_all_body_names_and_body_tree_str():
    generator = TestMJCFGeneratorBase()
    generator.init_xml_root()
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
