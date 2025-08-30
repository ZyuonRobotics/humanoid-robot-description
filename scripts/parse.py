import click
from pathlib import Path

from hurodes.parsers import HumanoidMJCFParser, HumanoidURDFParser
from hurodes import MJCF_ROBOTS_PATH, ROBOTS_PATH

@click.command()
@click.option("--input-path", prompt='Input path', type=str, help="Input path")
@click.option("--robot-name", prompt='Robot name', type=str, help="Robot name")
@click.option("--base-link-name", prompt='Base link name', type=str, help="Base link name", default="base_link")
@click.option("--format-type", prompt='Format type', type=str, help="Format type", default="urdf")
def main(input_path, robot_name, format_type, base_link_name):
    # remove quotes from input_path
    input_path = input_path.strip("'")
    
    if format_type == "mjcf":
        parser = HumanoidMJCFParser(input_path)
    elif format_type == "urdf":
        parser = HumanoidURDFParser(input_path)
    else:
        raise ValueError(f"Invalid format type: {format_type}")

    save_path = Path(ROBOTS_PATH) / robot_name

    parser.print_body_tree()
    parser.parse(base_link_name)
    parser.save(save_path)

if __name__ == "__main__":
    main()