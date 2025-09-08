import click
from pathlib import Path

import mujoco
import mujoco.viewer

from hurodes.generators.mjcf_generator.mjcf_humanoid_generator import MJCFHumanoidGenerator
from hurodes.generators.urdf_generator.urdf_humanoid_generator import URDFHumanoidGenerator
from hurodes import ROBOTS_PATH

@click.command()
@click.argument("robot-name", type=str)
@click.option("--format-type", type=str, default="mjcf", help="Format type", prompt="Format type")
@click.option("--mujoco-urdf/--not-mujoco-urdf", type=bool, default=False, help="Whether to generate MuJoCo URDF")
def main(robot_name, format_type, mujoco_urdf):
    hrdf_path = Path(ROBOTS_PATH) / robot_name

    if format_type == "mjcf":
        generator = MJCFHumanoidGenerator.from_hrdf_path(hrdf_path)
        xml_string = generator.export(hrdf_path / "robot.xml")
    elif format_type == "urdf":
        generator = URDFHumanoidGenerator.from_hrdf_path(hrdf_path)
        xml_string = generator.export(hrdf_path / "robot.urdf", mujoco_urdf=mujoco_urdf)
    else:
        raise ValueError(f"Invalid format type: {format_type}")

    if format_type == "mjcf" or (mujoco_urdf and format_type == "urdf"):
        # Only apply mesh directory replacement for MJCF
        xml_string = xml_string.replace(
            'meshdir="meshes"', 
            f'meshdir="{Path(ROBOTS_PATH) / robot_name / "meshes"}"'
        )

        # Launch MuJoCo viewer for MJCF format
        m = mujoco.MjModel.from_xml_string(xml_string) # type: ignore
        d = mujoco.MjData(m) # type: ignore
        with mujoco.viewer.launch_passive(m, d) as viewer:
            while viewer.is_running():
                mujoco.mj_step(m, d) # type: ignore
                viewer.sync()

if __name__ == "__main__":
    main()
