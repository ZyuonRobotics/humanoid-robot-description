import xml.etree.ElementTree as ET

from hurodes.generators.mjcf_generator.mjcf_generator_base import MJCFGeneratorBase
from hurodes.generators.hrdf_mixin import HRDFMixin
from hurodes.utils.string import get_prefix_name

MUJOCO_SENSOR_NAME = {
    "linacc": "accelerometer",
    "angvel": "gyro",
    "quat": "framequat"
}

class MJCFHumanoidGenerator(HRDFMixin, MJCFGeneratorBase):
    """
    MJCF generator for humanoid robots.
    
    This class combines HRDFMixin for common humanoid functionality
    with MJCFGeneratorBase for MJCF-specific XML generation.
    """
    
    def __init__(self):
        super().__init__()

    def generate_single_body_xml(self, parent_body, body_idx, prefix=None):
        """
        Generate XML for a single body element.
        
        Args:
            parent_body: Parent XML element to attach the body to
            body_idx: Index of the body in the info list
            prefix: Optional prefix for naming elements
            
        Returns:
            The created body XML element
        """
        body_info_list = self.info_list("body")
        body_info = body_info_list[body_idx]
        body_name = body_info["name"].data
        
        # Create body element with attributes
        body_elem = ET.SubElement(parent_body, 'body', attrib=body_info.to_mujoco_dict("body", prefix=prefix))
        ET.SubElement(body_elem, 'inertial', attrib=body_info.to_mujoco_dict("inertial", prefix=prefix))

        # Add joint (freejoint for root body, regular joint for others)
        if parent_body.tag == "worldbody":
            ET.SubElement(body_elem, 'freejoint')
        else:
            joint_info = self.get_info_by_attr("body_name", body_name, "joint", single=True)
            ET.SubElement(body_elem, 'joint', attrib=joint_info.to_mujoco_dict(prefix=prefix))

        # Add mesh geometries
        mesh_info_list = self.get_info_by_attr("body_name", body_name, "mesh")
        for mesh_info in mesh_info_list:
            ET.SubElement(body_elem, 'geom', attrib=mesh_info.to_mujoco_dict(prefix=prefix))

        # Add simple geometries
        simple_geom_info_list = self.get_info_by_attr("body_name", body_name, "simple_geom")
        for simple_geom_info in simple_geom_info_list:
            ET.SubElement(body_elem, 'geom', attrib=simple_geom_info.to_mujoco_dict(prefix=prefix))
            
        return body_elem

    def recursive_generate_body(self, parent=None, current_index=-1, prefix=None):
        """
        Recursively generate body elements in the XML tree.
        
        Args:
            parent: Parent XML element (defaults to worldbody)
            current_index: Current body index in the hierarchy
            prefix: Optional prefix for naming elements
        """
        if parent is None:
            parent = self.get_elem("worldbody")

        for child_index, parent_idx in enumerate(self.body_parent_id):
            if parent_idx == current_index:
                body_elem = self.generate_single_body_xml(parent, child_index, prefix=prefix)
                self.recursive_generate_body(body_elem, child_index, prefix=prefix)

    def add_compiler(self, relative_mesh_path=True):
        """Add compiler configuration with mesh directory."""
        self.get_elem("compiler").attrib = {
            "angle": "radian",
            "autolimits": "true",
            "meshdir": "../meshes" if relative_mesh_path else str(self.mesh_directory)
        }
    
    def add_mesh(self, prefix=None):
        """
        Add mesh assets to the MJCF.
        
        Args:
            prefix: Optional prefix for mesh names
        """
        asset_elem = self.get_elem("asset")
        mesh_name_set = set()
        
        mesh_info_list = self.info_list("mesh")
        for mesh_info in mesh_info_list:
            mesh_name = mesh_info["name"].data
            if mesh_name in mesh_name_set:
                continue

            self.validate_mesh_exists(mesh_name)
            
            ET.SubElement(asset_elem, 'mesh', attrib={
                "name": get_prefix_name(prefix, mesh_name), 
                "file": f"{mesh_name}.{self.mesh_file_type}"
            })
            mesh_name_set.add(mesh_name)

    def add_actuator(self, prefix=None):
        """
        Add actuators for joints.
        
        Args:
            prefix: Optional prefix for actuator names
        """
        actuator_info_list = self.info_list("actuator")
        if len(actuator_info_list) == 0:
            return
            
        actuator_elem = ET.SubElement(self.xml_root, 'actuator')
        
        # Keep the order of joints
        joint_info_list = self.info_list("joint")
        for joint_info in joint_info_list:
            actuator_info = self.get_info_by_attr("joint_name", joint_info["name"].data, "actuator", single=True)
            ET.SubElement(actuator_elem, 'motor', attrib=actuator_info.to_mujoco_dict(prefix=prefix))

    def add_sensors(self, prefix=None):
        """
        Add sensors for the robot.
        
        Args:
            prefix: Optional prefix for sensor names
        """
        self.add_imu(prefix=prefix)

        sensor_elem = self.get_elem("sensor")
        for body_info in self.info_list("body"):
            body_name = get_prefix_name(prefix, body_info["name"].data)
            ET.SubElement(sensor_elem, "framelinvel", attrib={"objtype":"xbody", "objname": body_name})
            ET.SubElement(sensor_elem, "frameangvel", attrib={"objtype":"xbody", "objname": body_name})

    def add_imu(self, prefix=None):
        """
        Add IMUs for bodies.
        
        Args:
            prefix: Optional prefix for IMU names
        """
        for imu_config in self.imu_configs:
            if imu_config.has_none: # skip config containing none
                continue
            found_body = False
            for body_elem in self.get_elem("worldbody").findall("body"):
                if body_elem.attrib["name"] == get_prefix_name(prefix, imu_config.body_name):
                    ET.SubElement(body_elem, 'site', attrib={
                        "name": get_prefix_name(prefix, imu_config.name),
                        "pos": " ".join([str(x) for x in imu_config.position]),
                        "quat": " ".join([str(x) for x in imu_config.orientation]),
                    })
                    found_body = True
                    break
            assert found_body, f"Body {imu_config.body_name} not found in the MJCF file."
            sensor_elem = self.get_elem("sensor")
            for value in imu_config.value:
                sensor_name = MUJOCO_SENSOR_NAME[value]
                site_name = get_prefix_name(prefix, imu_config.name)
                attrib = {"name": get_prefix_name(prefix, f"{imu_config.name}_{value}")}
                if sensor_name == "framequat":
                    attrib["objtype"] = "site"
                    attrib["objname"] = site_name
                else:
                    attrib["site"] = site_name
                ET.SubElement(sensor_elem, sensor_name, attrib)


    def _generate(self, prefix=None, add_scene=True, relative_mesh_path=True):
        """
        Generate the complete MJCF for the humanoid robot.
        
        Args:
            prefix: Optional prefix for element names
            add_scene: Whether to add scene elements
        """
        self.add_compiler(relative_mesh_path=relative_mesh_path)
        self.add_mesh(prefix=prefix)
        self.recursive_generate_body(prefix=prefix)
        self.add_actuator(prefix=prefix)
        self.add_sensors(prefix=prefix)
        if add_scene:
            self.add_scene()
