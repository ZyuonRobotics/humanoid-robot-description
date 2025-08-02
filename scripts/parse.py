import os
import click

from hurodes.format_parser import UnifiedMJCFParser, UnifiedURDFParser
from hurodes import MJCF_ROBOTS_PATH, ROBOTS_PATH

@click.command()
@click.option("--input_path", prompt='Input path', type=str, help="Path to the input MJCF/URDF file.")
@click.option("--robot_name", prompt='Robot name', type=str, help="Name of the robot.")
@click.option("--base_link_name", prompt='Base link name', type=str, help="Name of the base link.", default="base_link")
@click.option("--format_type", prompt='Format type', type=str, help="Format type of the input file.", default="urdf")
def main(input_path, robot_name, format_type, base_link_name):
    # remove quotes from input_path
    input_path = input_path.strip("'")
    
    if format_type == "mjcf":
        parser = UnifiedMJCFParser(input_path)
    elif format_type == "urdf":
        parser = UnifiedURDFParser(input_path)
    else:
        raise ValueError(f"Invalid format type: {format_type}")

    save_path = os.path.join(ROBOTS_PATH, robot_name)

    parser.print_body_tree()
    parser.parse(base_link_name)
    parser.save(save_path)

if __name__ == "__main__":
    main()