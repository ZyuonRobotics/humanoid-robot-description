import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco

from hurodes.format_parser.base_parser import BaseParser

class UnifiedURDFParser(BaseParser):
    def __init__(self, mjcf_path):
        super().__init__(mjcf_path)
        self.tree = ET.parse(self.file_path)
        self.root = self.tree.getroot()

    def fix_urdf(self, base_link_name="base_link"):
        # Ensure the root element is a robot tag
        if self.root.tag != 'robot':
            raise ValueError("Root element is not 'robot'")
        
        # Create mujoco tag
        if self.root.find("mujoco") is None:
            mujoco_elem = ET.Element('mujoco')
            ET.SubElement(
                mujoco_elem, 
                'compiler', {'meshdir': str(Path(self.file_path).parent.parent / "meshes"), 
                'balanceinertia': 'true', 
                'discardvisual': 'false'}
            )
            self.root.insert(0, mujoco_elem)
        
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

            dummy_link = ET.Element('link', {'name': 'dummy_link'})

            dummy_joint = ET.Element('joint', {'name': 'dummy_to_base_link', 'type': 'floating'})
            ET.SubElement(dummy_joint, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
            ET.SubElement(dummy_joint, 'parent', {'link': 'dummy_link'})
            ET.SubElement(dummy_joint, 'child', {'link': base_link_name})

            self.root.insert(0, dummy_link)
            self.root.insert(1, dummy_joint)

    def fix_actuator(self):
        for joint in self.root.findall("joint"):
            limit = joint.find("limit")
            if limit is not None and "effort" in limit.attrib:
                self.model_dict["actuator"].append({
                    "name": f"{joint.attrib['name']}_motor",
                    "joint": joint.attrib['name'],
                    "ctrlrange0": -float(limit.attrib['effort']),
                    "ctrlrange1": float(limit.attrib['effort']),
                })

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
