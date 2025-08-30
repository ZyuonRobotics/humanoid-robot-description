import click
from pathlib import Path

import mujoco
import mujoco.viewer

from hurodes.generators.mjcf_generator.humanoid_generator import HumanoidMJCFGenerator
from hurodes.generators.urdf_generator.humanoid_generator import HumanoidURDFGenerator
from hurodes import MJCF_ROBOTS_PATH, ROBOTS_PATH

@click.command()
@click.option("--robot-name", prompt='Robot name', type=str, help="Robot name")
@click.option("--format-type", prompt='Format type', type=str, help="Format type", default="mjcf")
def main(robot_name, format_type):
    hrdf_path = Path(ROBOTS_PATH) / robot_name

    if format_type == "mjcf":
        generator = HumanoidMJCFGenerator(hrdf_path)
        output_filename = "robot.xml"
    elif format_type == "urdf":
        generator = HumanoidURDFGenerator(hrdf_path)
        output_filename = "robot.urdf"
    else:
        raise ValueError(f"Invalid format type: {format_type}")

    xml_string = generator.export(hrdf_path / output_filename)

    if format_type == "mjcf":
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
    elif format_type == "urdf":
        print(f"URDF generated successfully: {hrdf_path / output_filename}")
        print("Note: URDF format does not support MuJoCo viewer. Use RViz or other URDF-compatible viewers.")

if __name__ == "__main__":
    main()