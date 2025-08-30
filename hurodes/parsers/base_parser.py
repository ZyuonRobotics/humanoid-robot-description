from pathlib import Path
from abc import ABC

import pandas as pd
import mujoco

from hurodes.hrdf.infos import ActuatorInfo, BodyInfo, SimpleGeomInfo, JointInfo, MeshInfo
from hurodes.hrdf.hrdf import HRDF, SimulatorConfig

PLANE_TYPE = int(mujoco.mjtGeom.mjGEOM_PLANE)
MESH_TYPE = int(mujoco.mjtGeom.mjGEOM_MESH)

class BaseParser(ABC):
    def __init__(self, file_path, robot_name):
        self.file_path = file_path
        self.robot_name = robot_name
        self.hrdf = HRDF()

        self.mesh_path = {}
        self.simulator_dict = {}

    @property
    def mujoco_spec(self):
        raise NotImplementedError("Subclasses must implement this method")

    def collect_body_info(self, model, spec):
        assert model.body(0).name == "world", "First body should be world."
        for body_idx in range(1, model.nbody):
            body = model.body(body_idx)
            body_info = BodyInfo.from_mujoco(body, spec.bodies[body_idx], model, spec)
            self.hrdf.info_list["body"].append(body_info)

    def collect_joint_info(self, model, spec):
        assert model.joint(0).type[0] == 0, "First joint should be free."
        for jnt_idx in range(1, model.njnt):
            joint_info = JointInfo.from_mujoco(model.joint(jnt_idx), spec.joints[jnt_idx], model, spec)
            self.hrdf.info_list["joint"].append(joint_info)

    def collect_actuator_info(self, model, spec):
        for actuator_idx in range(model.nu):
            actuator_info = ActuatorInfo.from_mujoco(model.actuator(actuator_idx), spec.actuators[actuator_idx], model, spec)
            self.hrdf.info_list["actuator"].append(actuator_info)

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
                mesh_info = MeshInfo.from_mujoco(geom_model, geom_spec, model, spec)
                self.hrdf.info_list["mesh"].append(mesh_info)
            else:
                geom_info = SimpleGeomInfo.from_mujoco(geom_model, geom_spec, model, spec)
                self.hrdf.info_list["simple_geom"].append(geom_info)
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
        self.hrdf.simulator_config = SimulatorConfig.from_dict(self.simulator_dict)

    def save(self, max_faces=8000):
        """Save the parsed robot data using HumanoidRobot's save method."""
        self.hrdf.save(mesh_path=self.mesh_path, max_faces=max_faces)

    def print_body_tree(self, colorful=False):
        pass
