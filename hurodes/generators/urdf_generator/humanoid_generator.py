from pathlib import Path
import xml.etree.ElementTree as ET

from hurodes.generators.urdf_generator.urdf_generator_base import URDFGeneratorBase
from hurodes.hrdf.hrdf import HumanoidRobot

def get_prefix_name(prefix, name):
    return f"{prefix}_{name}" if prefix else name

class HumanoidURDFGenerator(URDFGeneratorBase):
    def __init__(
            self,
            hrdf_path,
            robot_name=None
    ):
        # Use directory name as robot name if not specified
        if robot_name is None:
            robot_name = Path(hrdf_path).name
        super().__init__(robot_name)
        self.hrdf_path = hrdf_path
        self.humanoid_robot = None

    def load(self):
        self.humanoid_robot = HumanoidRobot.from_dir(self.hrdf_path)

    def generate(self):
        link_dict = {}
        for body_info in self.humanoid_robot.info_list["body"]:
            link_elem = ET.SubElement(self.xml_root, "link")
            body_name = body_info["name"].data
            link_dict[body_name] = link_elem

            body_info.to_urdf_elem(link_elem, "link")

            simple_geom_infos = self.humanoid_robot.find_info_by_attr("body_name", body_name, "simple_geom")
            for simple_geom_info in simple_geom_infos:
                simple_geom_info.to_urdf_elem(link_elem)

            mesh_infos = self.humanoid_robot.find_info_by_attr("body_name", body_name, "mesh")
            for mesh_info in mesh_infos:
                mesh_info.to_urdf_elem(link_elem)

        joint_dict = {}
        for joint_info in self.humanoid_robot.info_list["joint"]:
            joint_elem = ET.SubElement(self.xml_root, "joint")
            joint_name, body_name = joint_info["name"].data, joint_info["body_name"].data
            joint_dict[joint_name] = joint_elem

            joint_info.to_urdf_elem(joint_elem)

            body_info = self.humanoid_robot.find_info_by_attr("name", body_name, "body", single=True)
            body_info.to_urdf_elem(joint_elem, "joint")

            actuator_info = self.humanoid_robot.find_info_by_attr("joint_name", joint_name, "actuator", single=True)
            actuator_info.to_urdf_elem(joint_elem)
