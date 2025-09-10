import click
from pathlib import Path

import mujoco
import mujoco.viewer

from hurodes.humanoid_robot import HumanoidRobot
from hurodes import ROBOTS_PATH

@click.command()
@click.argument("robot-name", type=str)
@click.option("--format-type", type=str, default="mjcf", help="Format type", prompt="Format type")
@click.option("--mujoco-urdf/--not-mujoco-urdf", type=bool, default=False, help="Whether to generate MuJoCo URDF")
def main(robot_name, format_type, mujoco_urdf):
    # Create HumanoidRobot instance from robot name
    robot = HumanoidRobot.from_name(robot_name)
    
    # Define output path
    output_dir = Path(ROBOTS_PATH) / robot_name
    
    if format_type == "mjcf":
        output_path = output_dir / "exported" / "robot.xml"
        xml_string = robot.export_mjcf(output_path)
    elif format_type == "urdf":
        output_path = output_dir / "exported" / "robot.urdf"
        xml_string = robot.export_urdf(output_path, mujoco_urdf=mujoco_urdf)
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
