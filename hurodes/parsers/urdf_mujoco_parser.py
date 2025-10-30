import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco

from hurodes.parsers.base_parser import BaseParser
from hurodes.parsers.mjcf_parser import HumanoidMJCFParser
from hurodes.hrdf.infos.actuator import ActuatorInfo
from hurodes.utils.string import parse_inertia_file

def is_mesh_dir(dir_path):
    if not Path(dir_path).exists() or not Path(dir_path).is_dir():
        return False
    for file in Path(dir_path).iterdir():
        if file.is_file() and file.suffix.lower() in [".obj", ".stl"]: # exist obj or stl file
            return True
    return False

def find_mesh_dir(urdf_path, mesh_dir_path=None):
    urdf_path = Path(urdf_path)
    if mesh_dir_path is not None:
        mesh_dir_path = Path(mesh_dir_path)
        if is_mesh_dir(mesh_dir_path):
            return mesh_dir_path

    if is_mesh_dir(urdf_path.parent / "meshes"):
        return urdf_path.parent / "meshes"
    if is_mesh_dir(urdf_path.parent.parent / "meshes"):
        return urdf_path.parent.parent / "meshes"
    return None


class HumanoidURDFMujocoParser(HumanoidMJCFParser):
    def __init__(
        self, 
        urdf_path, 
        robot_name, 
        mesh_dir_path=None,
        timestep=0.001
    ):
        BaseParser.__init__(self, urdf_path, robot_name)
        self.mesh_dir_path = mesh_dir_path
        self.timestep = timestep
        self.tree = ET.parse(str(self.file_path))
        self.root = self.tree.getroot()

    def fix_urdf_mujoco_tag(self):
        mesh_dir = find_mesh_dir(self.file_path, self.mesh_dir_path)
        
        mujoco_elem = self.root.find("mujoco")
        if mujoco_elem is None:
            mujoco_elem = ET.Element('mujoco')
            
            assert mesh_dir is not None, "Mesh directory not found"
            ET.SubElement(mujoco_elem, 'compiler', attrib={
                'meshdir': str(mesh_dir),
                "balanceinertia": "true",
                "discardvisual": "false"
            })
            self.root.insert(0, mujoco_elem)
        else:
            compiler_elem = mujoco_elem.find("compiler")
            if compiler_elem is None:
                assert mesh_dir is not None, "Mesh directory not found"
                ET.SubElement(mujoco_elem, 'compiler', {'meshdir': str(mesh_dir)})
            else:
                original_mesh_dir = compiler_elem.attrib['meshdir']
                if is_mesh_dir(original_mesh_dir):
                    mesh_dir = Path(original_mesh_dir)
                elif is_mesh_dir(self.file_path.parent / original_mesh_dir):
                    mesh_dir = self.file_path.parent / original_mesh_dir
                else:
                    assert mesh_dir is not None, "Mesh directory not found"
                compiler_elem.attrib['meshdir'] = str(mesh_dir)

    def fix_urdf_worldbody(self, base_link_name="base_link"):
         # check if floating joint exists
        floating_joint_exists = False
        for joint in self.root.findall("joint"):
            if joint.attrib['type'] == 'floating':
                floating_joint_exists = True
                break
        if not floating_joint_exists:
            # check "base_link_name" is in the urdf
            base_link_exists = False
            for link in self.root.findall("link"):
                if link.attrib['name'] == base_link_name:
                    base_link_exists = True
                    break
            assert base_link_exists, f"{base_link_name} not found in the urdf"

            dummy_link = ET.Element('link', {'name': 'dummy-link'})

            dummy_joint = ET.Element('joint', {'name': 'dummy-joint', 'type': 'floating'})
            ET.SubElement(dummy_joint, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
            ET.SubElement(dummy_joint, 'parent', {'link': 'dummy-link'})
            ET.SubElement(dummy_joint, 'child', {'link': base_link_name})

            self.root.insert(0, dummy_link)
            self.root.insert(1, dummy_joint)

    def fix_urdf_inerita(self):
        for text_path in (self.file_path.parent.parent / "sw_inertia").iterdir():
            assert text_path.is_file(), f"{text_path} is not a file"
            
            with open(text_path, "r") as f:
                content = f.read()
            link_name, mass, inertia_dict = parse_inertia_file(content)
            link_found = False
            for link_elem in self.root.findall("link"):
                if link_elem.attrib['name'] == link_name:
                    link_found = True
                    inertial_elem = link_elem.find("inertial")
                    assert inertial_elem is not None, f"inertial element not found in the link {link_name}"

                    inertia_elem = inertial_elem.find("inertia")
                    assert inertia_elem is not None, f"inertia element not found in the link {link_name}"
                    inertia_elem.attrib.update({k: str(v) for k, v in inertia_dict.items()})
                    
                    mass_elem = inertial_elem.find("mass")
                    assert mass_elem is not None, f"mass element not found in the inertia element of the link {link_name}"
                    mass_elem.attrib["value"] = str(mass)
                    break
            assert link_found, f"{link_name} not found in the urdf"


    def fix_urdf(self, base_link_name="base_link"):
        # Ensure the root element is a robot tag
        if self.root.tag != 'robot':
            raise ValueError("Root element is not 'robot'")

        self.fix_urdf_mujoco_tag()
        self.fix_urdf_worldbody(base_link_name=base_link_name)
        sw_inertia_path = self.file_path.parent.parent / "sw_inertia"
        if sw_inertia_path.exists():
            self.fix_urdf_inerita()
        
    def fix_actuator(self):
        for joint in self.root.findall("joint"):
            if joint.attrib['type'] != 'revolute':
                continue
            limit = joint.find("limit")
            actuator_info_dict = {
                    "name": f"{joint.attrib['name']}_motor",
                    "joint_name": joint.attrib['name'],
                }
            if limit is not None:
                if "effort" in limit.attrib:
                    actuator_info_dict["peak_torque"] = float(limit.attrib['effort'])
                if "velocity" in limit.attrib:
                    actuator_info_dict["peak_velocity"] = float(limit.attrib['velocity'])

            actuator_info = ActuatorInfo.from_dict(actuator_info_dict)
            self.hrdf.add_info("actuator", actuator_info)

    @property
    def mujoco_spec(self):
        tree = ET.ElementTree(self.root)
        ET.indent(tree, space="  ", level=0)
        urdf_string = ET.tostring(self.root, encoding='unicode', method='xml')
        spec = mujoco.MjSpec.from_string(urdf_string) # type: ignore
        mjcf_string = spec.to_xml()
        spec = mujoco.MjSpec.from_string(mjcf_string) # type: ignore
        spec.option.timestep = self.timestep
        spec.compile()
        return spec

    def parse(self, base_link_name="base_link"):
        self.fix_urdf(base_link_name)
        super().parse()
        self.fix_actuator()

    def print_body_tree(self, colorful=False):
        print("print_body_tree Not implemented")
