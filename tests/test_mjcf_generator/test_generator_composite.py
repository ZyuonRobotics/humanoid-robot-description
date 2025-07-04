import os
from pathlib import Path
import xml.etree.ElementTree as ET

from fsspec.caching import P

from hurodes.mjcf_generator.generator_composite import MJCFGeneratorComposite
from hurodes.mjcf_generator.generator_base import MJCFGeneratorBase

class MockGenerator(MJCFGeneratorBase):
    def __init__(self, name, mesh_name=None, mesh_file=None, meshdir=None, **kwargs):
        super().__init__(**kwargs)
        self._name = name
        self._mesh_name = mesh_name or f"mesh_{name}"
        self._mesh_file = mesh_file or f"{name}.stl"
        self._meshdir = meshdir
        self.loaded = False
        self.generated = False

    def load(self):
        self.loaded = True

    def generate(self, prefix=None):
        self.generated = True
        # Add asset/mesh and compiler/meshdir for testing
        asset = self.get_elem("asset")
        ET.SubElement(asset, "mesh", name=self._mesh_name, file=self._mesh_file)
        compiler = self.get_elem("compiler")
        if self._meshdir:
            compiler.set("meshdir", self._meshdir)


def test_composite_init_and_load():
    g1 = MockGenerator("a")
    g2 = MockGenerator("b")
    composite = MJCFGeneratorComposite([g1, g2])
    assert set(composite.generators.keys()) == {"generator0", "generator1"}
    composite.load()
    assert g1.loaded and g2.loaded


def test_composite_generate_and_merge():
    g1 = MockGenerator("a", meshdir="/tmp/meshA")
    g2 = MockGenerator("b", meshdir="/tmp/meshB")
    composite = MJCFGeneratorComposite({"g1": g1, "g2": g2})
    composite.generate()

    asset = composite.get_elem("asset")
    mesh_names = {m.get("name") for m in asset.findall("mesh")}
    assert mesh_names == {g1._mesh_name, g2._mesh_name}

    meshdir = composite.get_elem("compiler").get("meshdir")
    expected_common = os.path.commonpath([str(g1._meshdir), str(g2._meshdir)])
    print(meshdir, expected_common)
    assert meshdir == expected_common

def test_get_mesh_path():
    g1 = MockGenerator("a", meshdir="/tmp/meshA")
    g2 = MockGenerator("b", meshdir="/tmp/meshB")
    composite = MJCFGeneratorComposite([g1, g2])

    composite._prepare_generators()
    path1 = composite.get_mesh_path(g1._mesh_name)
    path2 = composite.get_mesh_path(g2._mesh_name)
    assert path1 == Path("/tmp/meshA/a.stl")
    assert path2 == Path("/tmp/meshB/b.stl") 

    composite.destroy()
    composite.generate() # get_mesh_path should not be called after generate
    path1 = composite.get_mesh_path(g1._mesh_name) # wrong path
    path2 = composite.get_mesh_path(g2._mesh_name) # wrong path
    assert path1 == Path("/tmp/meshA/meshA/a.stl")
    assert path2 == Path("/tmp/meshB/meshB/b.stl") 

def test_composite_destroy():
    g1 = MockGenerator("a")
    g2 = MockGenerator("b")
    composite = MJCFGeneratorComposite([g1, g2])
    # Trigger xml_root creation
    _ = composite.xml_root
    _ = g1.xml_root
    _ = g2.xml_root
    # Ensure xml_root is not None before destroy
    assert composite._xml_root is not None
    assert g1._xml_root is not None
    assert g2._xml_root is not None
    # Call destroy
    composite.destroy()
    # After destroy, all _xml_root should be None
    assert composite._xml_root is None
    assert g1._xml_root is None
    assert g2._xml_root is None 
