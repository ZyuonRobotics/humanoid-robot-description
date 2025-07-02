from typing import Union, List, Dict
from hurodes.mjcf_generator.generator_base import MJCFGeneratorBase

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

    def generate(self):
        # TODO: deal with meshdir of different generators
        for name, generator in self.generators.items():
            generator.init_xml_root()
            generator.generate(prefix=name)
            for top_elem in generator.xml_root:
                last_top_elem = self.get_elem(top_elem.tag)
                last_top_elem.attrib |= top_elem.attrib
                for elem in top_elem:
                    last_top_elem.append(elem)

if __name__ == "__main__":
    import mujoco
    import mujoco.viewer
    from pathlib import Path

    from hurodes.mjcf_generator.unified_generator import UnifiedMJCFGenerator
    from hurodes import ROBOTS_PATH


    generator = MJCFGeneratorComposite({
        "robot1": UnifiedMJCFGenerator(Path(ROBOTS_PATH, "kuavo_s45")),
        "robot2": UnifiedMJCFGenerator(Path(ROBOTS_PATH, "kuavo_s45")),
    })
    generator.build()

    generator.export("test.xml")

    m = mujoco.MjModel.from_xml_string(generator.mjcf_str) # type: ignore
    d = mujoco.MjData(m) # type: ignore
    with mujoco.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            mujoco.mj_step(m, d) # type: ignore
            viewer.sync()