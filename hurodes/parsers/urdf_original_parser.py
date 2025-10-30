import xml.etree.ElementTree as ET
from pathlib import Path

from hurodes.parsers.base_parser import BaseParser
from hurodes.hrdf.infos import ActuatorInfo, BodyInfo, SimpleGeomInfo, JointInfo, MeshInfo, simple_geom
from hurodes.hrdf.hrdf import SimulatorConfig
from hurodes.utils.convert import rpy2quat
from hurodes.parsers.urdf_mujoco_parser import find_mesh_dir

class LinkNode:
    def __init__(self, name):
        self.name = name
        self.joint_name = None
        self.parent = None
        self.children = []
        self.id = None
        self.pos = None
        self.quat = None

    def __repr__(self):
        children_names = [child.name for child in self.children]
        return f"LinkNode(name={self.name}, joint_name={self.joint_name}, parent={self.parent.name if self.parent else None}, children={children_names}, id={self.id})"

def check_and_assign_id(link_nodes):
    root = None
    for node in link_nodes.values():
        if node.parent is None:
            assert root is None, f"Multiple root nodes found: {root.name} and {node.name}"
            root = node
    
    if root is None:
        raise ValueError("No root node found")
    
    # BFS to check if the tree is connected and has no cycles
    visited = set()
    stack = [root]
    current_id = 0
    while stack:
        current = stack.pop()
        current.id = current_id
        current_id += 1
        assert current not in visited, f"Cycle detected: {current.name}"
        visited.add(current)
        
        for i in range(len(current.children) - 1, -1, -1):
            child = current.children[i]
            assert child in link_nodes.values(), f"Child node {child.name} not found in link_nodes"
            stack.append(child)
    return root

class HumanoidURDFOriginalParser(BaseParser):
    def __init__(self, urdf_path, robot_name, mesh_dir_path=None, timestep=0.001):
        super().__init__(urdf_path, robot_name)
        self.mesh_dir_path = mesh_dir_path
        self.timestep = timestep
        self.tree = ET.parse(str(self.file_path))
        self.root = self.tree.getroot()

        self.link_nodes = None
        self.root_link_node = None
        
        # Validate URDF structure
        assert self.root.tag == "robot", f"Root element must be 'robot', got '{self.root.tag}'"

    def build_kinematic_tree(self):
        """Build the kinematic tree from URDF joints"""
        joints = self.root.findall("joint")
        self.link_nodes: dict[str, LinkNode] = {}

        for joint in joints:
            if joint.get("type") == "floating": # from world body to base link, added for mujoco simulator
                pass
            elif joint.get("type") == "revolute":  # Skip floating joints
                joint_name = joint.get("name")
                parent_name = joint.find("parent").get("link")
                child_name = joint.find("child").get("link")

                if parent_name not in self.link_nodes:
                    self.link_nodes[parent_name] = LinkNode(parent_name)
                if child_name not in self.link_nodes:
                    self.link_nodes[child_name] = LinkNode(child_name)
                self.link_nodes[child_name].joint_name = joint_name

                self.link_nodes[parent_name].children.append(self.link_nodes[child_name])
                self.link_nodes[child_name].parent = self.link_nodes[parent_name]

                self.link_nodes[child_name].pos = [float(x) for x in joint.find("origin").get("xyz").split()]
                rpy = [float(x) for x in joint.find("origin").get("rpy").split()]
                self.link_nodes[child_name].quat = rpy2quat(rpy)
            else:
                raise ValueError(f"Invalid joint type: {joint.get('type')}")
        
        self.root_link_node = check_and_assign_id(self.link_nodes)

    def collect_body_info(self):
        for link in self.root.findall("link"):
            body_info = BodyInfo.from_urdf(link, self.root, link_nodes=self.link_nodes)
            self.hrdf.add_info("body", body_info)

            for geom in link.findall("visual") + link.findall("collision"):
                if geom.find("geometry").find("mesh") is not None:
                    mesh_info = MeshInfo.from_urdf(geom, self.root, body_name=body_info["name"].data)
                    self.hrdf.add_info("mesh", mesh_info)
                else:
                    simple_geom_info = SimpleGeomInfo.from_urdf(geom, self.root, body_name=body_info["name"].data)
                    self.hrdf.add_info("simple_geom", simple_geom_info)

    def collect_joint_info(self):
        for joint in self.root.findall("joint"):
            if joint.get("type") == "floating":
                pass
            elif joint.get("type") == "revolute":
                joint_info = JointInfo.from_urdf(joint, self.root)
                self.hrdf.add_info("joint", joint_info)

                actuator_info = ActuatorInfo.from_urdf(joint, self.root)
                self.hrdf.add_info("actuator", actuator_info)
            else:
                raise ValueError(f"Invalid joint type: {joint.get('type')}")

    def collect_mesh_path(self):
        mesh_file_types = []

        mesh_dir = find_mesh_dir(self.file_path, self.mesh_dir_path)

        for link in self.root.findall("link"):
            for geom in link.findall("visual") + link.findall("collision"):
                if geom.find("geometry").find("mesh") is not None:
                    mesh_path = geom.find("geometry").find("mesh").get("filename")
                    assert mesh_path is not None

                    mesh_file_name = mesh_path.split("/")[-1]
                    assert (mesh_dir / mesh_file_name).exists(), f"Mesh file {mesh_file_name} not found in {mesh_dir}"
                    mesh_name = mesh_file_name.split(".")[0].replace("-", "_")
                    mesh_type = mesh_file_name.split(".")[1].lower()

                    self.mesh_path[mesh_name] = mesh_dir / mesh_file_name
                    mesh_file_types.append(mesh_type)

        assert len(set(mesh_file_types)) == 1, "All mesh files must have the same file type."
        assert mesh_file_types[0] in ["obj", "stl"], "Mesh file type must be obj or stl."
        self.hrdf.mesh_file_type = mesh_file_types[0]

    def parse(self, base_link_name="base_link"):
        self.hrdf.robot_name = self.robot_name

        self.build_kinematic_tree()
        self.collect_body_info()
        self.collect_joint_info()
        self.collect_mesh_path()
        self.hrdf.fix_simple_geom()

        body_parent_id = [0] * len(self.link_nodes)
        for node in self.link_nodes.values():
            if node.parent is not None:
                body_parent_id[node.id] = node.parent.id
            else:
                body_parent_id[node.id] = -1
        self.hrdf.body_parent_id = body_parent_id

        self.hrdf.simulator_config = SimulatorConfig(timestep=self.timestep)

        self.parse_body_name()
        self.parse_imu()

    def print_body_tree(self, colorful=False):
        print("print_body_tree Not implemented")
