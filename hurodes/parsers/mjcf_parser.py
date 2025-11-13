import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Union

import mujoco

from hurodes.parsers.base_parser import BaseParser
from hurodes.utils.string import get_elem_tree_str
from hurodes.hrdf.infos import ActuatorInfo, BodyInfo, SimpleGeomInfo, JointInfo, MeshInfo
from hurodes.hrdf.hrdf import SimulatorConfig

PLANE_TYPE = int(mujoco.mjtGeom.mjGEOM_PLANE)
MESH_TYPE = int(mujoco.mjtGeom.mjGEOM_MESH)


class HumanoidMJCFParser(BaseParser):

    def __init__(self, mjcf_path: Union[Path, str], robot_name: str):
        super().__init__(mjcf_path, robot_name)
        self.tree = ET.parse(str(self.file_path))
        self.root = self.tree.getroot()
        self.worldbody = self.root.find("worldbody")
        assert self.worldbody is not None, "No <worldbody> element found in the MJCF file."
        root_bodies = self.worldbody.findall("body")
        assert len(root_bodies) == 1, "There should be exactly one root <body> element in the <worldbody> element."
        self.base_link = root_bodies[0]

    def print_body_tree(self, colorful=False):
        print(get_elem_tree_str(self.base_link, colorful=colorful))

    @property
    def mujoco_spec(self):
        return mujoco.MjSpec.from_file(str(self.file_path)) # type: ignore

    def collect_body_info(self, model, spec):
        assert model.body(0).name == "world", "First body should be world."
        for body_idx in range(1, model.nbody):
            body = model.body(body_idx)
            body_info = BodyInfo.from_mujoco(body, spec.bodies[body_idx])
            self.hrdf.add_info("body", body_info)

    def collect_joint_info(self, model, spec):
        assert model.joint(0).type[0] == 0, "First joint should be free."
        for jnt_idx in range(1, model.njnt):
            joint_info = JointInfo.from_mujoco(model.joint(jnt_idx), spec.joints[jnt_idx], whole_spec=spec)
            self.hrdf.add_info("joint", joint_info)

    def collect_actuator_info(self, model, spec):
        for actuator_idx in range(model.nu):
            actuator_info = ActuatorInfo.from_mujoco(model.actuator(actuator_idx), spec.actuators[actuator_idx])
            self.hrdf.add_info("actuator", actuator_info)

    def collect_geom_info(self, model, spec):
        ground_dict = None
        for geom_idx in range(model.ngeom):
            geom_model, geom_spec = model.geom(geom_idx), spec.geoms[geom_idx]

            if geom_model.bodyid[0] == 0: # geom in worldbody
                assert ground_dict is None, "Only one plane is allowed."
                assert int(geom_model.type) == PLANE_TYPE, "Plane should be of type plane."
                ground_dict = {
                    "contact_type": str(geom_model.contype[0]),
                    "contact_affinity": str(geom_model.conaffinity[0]),
                    "friction": str(geom_model.friction[0]),
                    "type": "plane",
                }
                continue

            if int(geom_model.type) == MESH_TYPE:
                mesh_info = MeshInfo.from_mujoco(geom_model, geom_spec, whole_spec=spec)
                self.hrdf.add_info("mesh", mesh_info)
            else:
                geom_info = SimpleGeomInfo.from_mujoco(geom_model, geom_spec, whole_spec=spec)
                self.hrdf.add_info("simple_geom", geom_info)

        if ground_dict is None: # maybe from urdf, use default result
            self.simulator_dict["ground"] = {
                "contact_affinity": 1,
                "contact_type": 1,
                "friction": 1.0,
                "type": "plane",
            }
        else:
            self.simulator_dict["ground"] = ground_dict

    def collect_mesh_path(self, spec):
        mesh_file_types = []

        meshdir = Path(spec.meshdir)
        if not meshdir.is_absolute():
            meshdir = (Path(self.file_path).parent / meshdir).resolve()
        assert meshdir.exists(), f"Mesh directory {meshdir} does not exist."

        for mesh in spec.meshes:
            self.mesh_path[mesh.name.replace("-", "_")] = meshdir /  mesh.file
            mesh_file_types.append(mesh.file.split('.')[-1].lower())

        assert len(set(mesh_file_types)) == 1, "All mesh files must have the same file type."
        assert mesh_file_types[0] in ["obj", "stl"], "Mesh file type must be obj or stl."
        self.hrdf.mesh_file_type = mesh_file_types[0]

    def parse(self, base_link_name="base_link"):
        spec = self.mujoco_spec
        model = spec.compile()

        self.hrdf.robot_name = self.robot_name

        self.simulator_dict["timestep"] = spec.option.timestep
        self.simulator_dict["gravity"] = spec.option.gravity

        self.hrdf.body_parent_id = (model.body_parentid[1:] - 1).tolist()
        self.collect_body_info(model, spec)
        self.collect_joint_info(model, spec)
        self.collect_actuator_info(model, spec)
        self.collect_geom_info(model, spec)
        self.collect_mesh_path(spec)
        self.hrdf.fix_simple_geom()
        self.hrdf.simulator_config = SimulatorConfig.from_dict(self.simulator_dict)

        self.parse_body_name()
        self.parse_imu()