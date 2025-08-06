import os
import click

import mujoco
import mujoco.viewer

from hurodes.mjcf_generator.unified_generator import UnifiedMJCFGenerator
from hurodes.mjcf_generator.generator_composite import MJCFGeneratorComposite
from hurodes import ROBOTS_PATH

@click.command()
@click.option(
    "--robot-names",
    prompt="Robot names (comma-separated)",
    type=str,
    help="Robot names (comma-separated)"
)
def main(robot_names):
    robot_names_list = [name.strip() for name in robot_names.split(",") if name.strip()]
    if len(robot_names_list) < 2:
        raise click.UsageError("Please provide at least two robot names for composition.")

    generators = [UnifiedMJCFGenerator(os.path.join(ROBOTS_PATH, name)) for name in robot_names_list]
    generator = MJCFGeneratorComposite(generators)
    generator.build()

    m = mujoco.MjModel.from_xml_string(generator.mjcf_str)  # type: ignore
    d = mujoco.MjData(m)  # type: ignore
    with mujoco.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            mujoco.mj_step(m, d)  # type: ignore
            viewer.sync()

if __name__ == "__main__":
    main() 