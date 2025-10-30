import click
from pathlib import Path

from hurodes.parsers import HumanoidMJCFParser, HumanoidURDFMujocoParser, HumanoidURDFOriginalParser

@click.command()
@click.argument("input-path", type=str)
@click.argument("robot-name", type=str)
@click.option("--format-type", type=str, default="urdf-mujoco", help="Format type", prompt="Format type")
@click.option("--base-link-name", prompt='Base link name', type=str, help="Base link name", default="base_link")
def main(input_path, robot_name, format_type, base_link_name):
    # remove quotes from input_path
    input_path = input_path.strip("'")
    
    if format_type == "mjcf":
        parser = HumanoidMJCFParser(input_path, robot_name)
    elif format_type == "urdf-mujoco":
        parser = HumanoidURDFMujocoParser(input_path, robot_name)
    elif format_type == "urdf-original":
        parser = HumanoidURDFOriginalParser(input_path, robot_name)
    else:
        raise ValueError(f"Invalid format type: {format_type}")

    parser.parse(base_link_name)
    parser.print_body_tree()
    parser.save()

if __name__ == "__main__":
    main()
