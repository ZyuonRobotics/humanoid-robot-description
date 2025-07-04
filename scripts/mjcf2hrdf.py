import os
import click

from hurodes.mjcf_parser.unified_parser import UnifiedMJCFParser
from hurodes import MJCF_ROBOTS_PATH, ROBOTS_PATH

@click.command()
@click.option("--mjcf_path", prompt='MJCF path', type=str, help="Path to the input MJCF file.")
@click.option("--robot_name", prompt='Robot name', type=str, help="Name of the robot.")
def main(mjcf_path, robot_name):
    parser = UnifiedMJCFParser(mjcf_path)
    save_path = os.path.join(ROBOTS_PATH, robot_name)

    parser.print_body_tree()
    parser.parse()
    parser.save(save_path)

if __name__ == "__main__":
    main()