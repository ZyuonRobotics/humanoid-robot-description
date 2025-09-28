import click
from pathlib import Path

import mujoco
import mujoco.viewer

from hurodes.humanoid_robot import HumanoidRobot
from hurodes import ROBOTS_PATH

@click.command()
@click.argument("robot-name", type=str)
@click.option("--format-type", type=str, default="mjcf", help="Format type", prompt="Format type")
@click.option("--add-mujoco-tag/--not-add-mujoco-tag", type=bool, default=False, help="Whether to add MuJoCo tag to URDF")
@click.option("--view/--not-view", type=bool, default=True, help="Whether to view the model")
def main(robot_name, format_type, add_mujoco_tag, view):
    # Create HumanoidRobot instance from robot name
    robot = HumanoidRobot.from_name(robot_name)
    
    # Define output path
    output_dir = Path(ROBOTS_PATH) / robot_name
    
    if format_type == "mjcf":
        output_path = output_dir / "exported" / "robot.xml"
        robot.export_mjcf(output_path)
    elif format_type == "urdf":
        output_path = output_dir / "exported" / "robot.urdf"
        robot.export_urdf(output_path, add_mujoco_tag=add_mujoco_tag)
    else:
        raise ValueError(f"Invalid format type: {format_type}")

    if view:
        if format_type == "mjcf":
            generator = robot.build_mjcf_generator()
            generator.generate(relative_mesh_path=False)
        elif format_type == "urdf":
            assert add_mujoco_tag, "MuJoCo tag is required to view URDF in MuJoCo viewer"
            generator = robot.build_urdf_generator()
            generator.generate(relative_mesh_path=False, add_mujoco_tag=add_mujoco_tag)
        else:
            raise ValueError(f"Invalid format type: {format_type}")
        m = mujoco.MjModel.from_xml_string(generator.xml_str) # type: ignore
        d = mujoco.MjData(m) # type: ignore
        with mujoco.viewer.launch_passive(m, d) as viewer:
            while viewer.is_running():
                mujoco.mj_step(m, d) # type: ignore
                viewer.sync()

if __name__ == "__main__":
    main()
