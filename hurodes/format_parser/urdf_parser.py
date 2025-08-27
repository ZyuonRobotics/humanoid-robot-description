import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco

from hurodes.format_parser.base_parser import BaseParser
from hurodes.hrdf.actuator import ActuatorInfos

class UnifiedURDFParser(BaseParser):
    def __init__(self, mjcf_path, mesh_dir_path=None):
        super().__init__(mjcf_path)
        self.mesh_dir_path = mesh_dir_path

        self.tree = ET.parse(self.file_path)
        self.root = self.tree.getroot()

    def is_mesh_dir(self, dir_path):
        if not Path(dir_path).exists():
            return False
        if not Path(dir_path).is_dir():
            return False
        for file in Path(dir_path).iterdir():
            if file.is_file() and file.suffix.lower() in [".obj", ".stl"]:
                return True
        return False
        

    def fix_urdf(self, base_link_name="base_link"):
        # Ensure the root element is a robot tag
        if self.root.tag != 'robot':
            raise ValueError("Root element is not 'robot'")

        # meshdir
        if self.mesh_dir_path is not None:
            if self.is_mesh_dir(self.mesh_dir_path):
                mesh_dir = Path(self.mesh_dir_path)
            else:
                raise ValueError(f"Mesh directory {self.mesh_dir_path} is not a valid mesh directory")
        elif self.is_mesh_dir(Path(self.file_path).parent / "meshes"):
            mesh_dir = Path(self.file_path).parent / "meshes"
        elif self.is_mesh_dir(Path(self.file_path).parent.parent / "meshes"):
            mesh_dir = Path(self.file_path).parent.parent / "meshes"
        else:
            mesh_dir = None
        
        # Create mujoco tag
        mujoco_elem = self.root.find("mujoco")
        if mujoco_elem is None:
            mujoco_elem = ET.Element('mujoco')
            
            assert mesh_dir is not None, "Mesh directory not found"
            ET.SubElement(mujoco_elem, 'compiler', {'meshdir': str(mesh_dir)})
            self.root.insert(0, mujoco_elem)
        else:
            compiler_elem = mujoco_elem.find("compiler")
            if compiler_elem is None:
                assert mesh_dir is not None, "Mesh directory not found"
                ET.SubElement(mujoco_elem, 'compiler', {'meshdir': str(mesh_dir)})
            else:
                original_mesh_dir = compiler_elem.attrib['meshdir']
                if self.is_mesh_dir(original_mesh_dir):
                    mesh_dir = Path(original_mesh_dir)
                elif self.is_mesh_dir(Path(self.file_path).parent / original_mesh_dir):
                    mesh_dir = Path(self.file_path).parent / original_mesh_dir
                else:
                    assert mesh_dir is not None, "Mesh directory not found"
                compiler_elem.attrib['meshdir'] = str(mesh_dir)
        
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

    def fix_actuator(self):
        # Initialize actuator_infos_list if it doesn't exist
        if self.actuator_infos_list is None:
            self.actuator_infos_list = []
            
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

            actuator_infos = ActuatorInfos.from_dict(actuator_info_dict)
            self.actuator_infos_list.append(actuator_infos)

    @property
    def mujoco_spec(self):
        tree = ET.ElementTree(self.root)
        ET.indent(tree, space="  ", level=0)
        urdf_string = ET.tostring(self.root, encoding='unicode', method='xml')
        spec = mujoco.MjSpec.from_string(urdf_string) # type: ignore
        spec.compile()
        return spec

    def parse(self, base_link_name="base_link"):
        self.fix_urdf(base_link_name)
        super().parse()
        self.fix_actuator()

    def print_body_tree(self, colorful=False):
        print("print_body_tree Not implemented")
