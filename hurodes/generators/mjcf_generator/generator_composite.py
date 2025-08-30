from typing import Union, List, Dict
from pathlib import Path
import os

from hurodes.generators.mjcf_generator.mjcf_generator_base import MJCFGeneratorBase


class MJCFGeneratorComposite(MJCFGeneratorBase):
    def __init__(self, generators: Union[List, Dict], **kwargs):
        super().__init__(**kwargs)
        if isinstance(generators, dict):
            self.generators = generators
        elif isinstance(generators, list):
            self.generators = {f"generator{i}": g for i, g in enumerate(generators)}
        else:
            raise NotImplementedError

    def load(self):
        for generator in self.generators.values():
            generator.load()

    def generate(self, prefix=None):
        self._prepare_generators(prefix)
        self._merge_generators_xml()
        # post-processing
        self._unify_mesh_paths_and_meshdir()
    
    def destroy(self):
        for generator in self.generators.values():
            generator.destroy()
        super().destroy()

    def _prepare_generators(self, prefix=None):
        for name, generator in self.generators.items():
            generator.generate(prefix=f"{prefix}_{name}" if prefix is not None else name)

    def _merge_generators_xml(self):
        for generator in self.generators.values():
            for top_elem in generator.xml_root:
                last_top_elem = self.get_elem(top_elem.tag)
                last_top_elem.attrib |= top_elem.attrib
                for elem in top_elem:
                    last_top_elem.append(elem)

    def get_mesh_path(self, mesh_name) -> Path:
        for generator in self.generators.values():
            compiler_elem = generator.get_elem("compiler")
            meshdir = compiler_elem.get("meshdir")
            for mesh_elem in generator.get_elem("asset").findall("mesh"):
                if mesh_elem.get("name") == mesh_name:
                    if meshdir is None:
                        return Path(mesh_elem.get("file"))
                    else:
                        return Path(meshdir) / mesh_elem.get("file")
        raise ValueError(f"Mesh {mesh_name} not found")

    def _unify_mesh_paths_and_meshdir(self):
        # Find all mesh file absolute paths in the merged xml
        asset_elem = self.get_elem("asset")
        mesh_elems = asset_elem.findall("mesh")

        # check all mesh name is unique
        mesh_names = [mesh_elem.get("name") for mesh_elem in mesh_elems]
        assert  len(mesh_names) == len(set(mesh_names)), "Mesh names are not unique"

        mesh_paths = [self.get_mesh_path(mesh_name) for mesh_name in mesh_names]
        common_meshdir = Path(os.path.commonpath(mesh_paths))

        # Update all mesh file attributes to be relative to common_meshdir
        for mesh_elem, abs_path in zip(mesh_elems, mesh_paths):
            rel_path = abs_path.relative_to(common_meshdir)
            mesh_elem.set("file", str(rel_path))

        # Set meshdir in compiler
        self.get_elem("compiler").set("meshdir", str(common_meshdir))
