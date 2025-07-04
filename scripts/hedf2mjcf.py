import os
import click

import mujoco
import mujoco.viewer

from hurodes.mjcf_generator.unified_generator import UnifiedMJCFGenerator
from hurodes import MJCF_ROBOTS_PATH, ROBOTS_PATH

@click.command()
@click.option("--robot_name", prompt='Robot name', type=str, help="Name of the robot.")
def main(robot_name):
    hrdf_path = os.path.join(ROBOTS_PATH, robot_name)

    generator = UnifiedMJCFGenerator(hrdf_path)
    xml_string = generator.export(os.path.join(hrdf_path, "robot.xml"))

    m = mujoco.MjModel.from_xml_string(xml_string) # type: ignore
    d = mujoco.MjData(m) # type: ignore
    with mujoco.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            mujoco.mj_step(m, d) # type: ignore
            viewer.sync()

if __name__ == "__main__":
    main()