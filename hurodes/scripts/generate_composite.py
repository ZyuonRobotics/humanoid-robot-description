import click
from pathlib import Path

import mujoco
import mujoco.viewer

from hurodes.generators.mjcf_generator.mjcf_humanoid_generator import MJCFHumanoidGenerator
from hurodes.generators.mjcf_generator.mjcf_generator_composite import MJCFGeneratorComposite
from hurodes import ROBOTS_PATH

@click.command()
@click.argument("robot-names", type=str)
def main(robot_names):
    robot_names_list = [name.strip() for name in robot_names.split(",") if name.strip()]
    if len(robot_names_list) < 2:
        raise click.UsageError("Please provide at least two robot names for composition.")

    generators = [MJCFHumanoidGenerator.from_hrdf_path(Path(ROBOTS_PATH) / name) for name in robot_names_list]
    generator = MJCFGeneratorComposite(generators)
    generator.export(Path("composite.xml"))

    m = mujoco.MjModel.from_xml_string(generator.xml_str)  # type: ignore
    d = mujoco.MjData(m)  # type: ignore
    with mujoco.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            mujoco.mj_step(m, d)  # type: ignore
            viewer.sync()

if __name__ == "__main__":
    main() 