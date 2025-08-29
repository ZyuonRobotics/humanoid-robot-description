from pathlib import Path
from abc import ABC

import pandas as pd
import mujoco

from hurodes.hrdf.infos import ActuatorInfo, BodyInfo, SimpleGeomInfo, JointInfo, MeshInfo
from hurodes.hrdf.hrdf import HumanoidRobot

PLANE_TYPE = int(mujoco.mjtGeom.mjGEOM_PLANE)
MESH_TYPE = int(mujoco.mjtGeom.mjGEOM_MESH)

class BaseParser(ABC):
    def __init__(self, file_path):
        self.file_path = file_path
        self.humanoid_robot = HumanoidRobot()

        self.mesh_path = {}

    @property
    def mujoco_spec(self):
        raise NotImplementedError("Subclasses must implement this method")

    def collect_body_info(self, model, spec):
        assert model.body(0).name == "world", "First body should be world."
        for body_idx in range(1, model.nbody):
            body = model.body(body_idx)
            body_info = BodyInfo.from_mujoco(body, spec.bodies[body_idx], model, spec)
            self.humanoid_robot.info_list["body"].append(body_info)

    def collect_joint_info(self, model, spec):
        assert model.joint(0).type[0] == 0, "First joint should be free."
        for jnt_idx in range(1, model.njnt):
            joint_info = JointInfo.from_mujoco(model.joint(jnt_idx), spec.joints[jnt_idx], model, spec)
            self.humanoid_robot.info_list["joint"].append(joint_info)

    def collect_actuator_info(self, model, spec):
        for actuator_idx in range(model.nu):
            actuator_info = ActuatorInfo.from_mujoco(model.actuator(actuator_idx), spec.actuators[actuator_idx], model, spec)
            self.humanoid_robot.info_list["actuator"].append(actuator_info)

    def collect_geom_info(self, model, spec):
        ground_dict = None
        for geom_idx in range(model.ngeom):
            geom_model, geom_spec = model.geom(geom_idx), spec.geoms[geom_idx]
            if int(geom_model.type) == MESH_TYPE:
                mesh_info = MeshInfo.from_mujoco(geom_model, geom_spec, model, spec)
                self.humanoid_robot.info_list["mesh"].append(mesh_info)
            elif int(geom_model.type) == PLANE_TYPE:
                assert geom_model.bodyid == 0, "Plane should be in worldbody."
                assert ground_dict is None, "Only one plane is allowed."
                ground_dict = {
                    "contype": str(geom_model.contype[0]),
                    "conaffinity": str(geom_model.conaffinity[0]),
                    "static_friction": str(geom_model.friction[0]),
                    "dynamic_friction": str(geom_model.friction[0]),
                }
            else:
                geom_info = SimpleGeomInfo.from_mujoco(geom_model, geom_spec, model, spec)
                self.humanoid_robot.info_list["simple_geom"].append(geom_info)
        self.humanoid_robot.ground_dict = ground_dict

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
        self.humanoid_robot.mesh_file_type = mesh_file_types[0]

    def parse(self, base_link_name="base_link"):
        spec = self.mujoco_spec
        model = spec.compile()

        self.humanoid_robot.body_parent_id = (model.body_parentid[1:] - 1).tolist()
        self.collect_body_info(model, spec)
        self.collect_joint_info(model, spec)
        self.collect_actuator_info(model, spec)
        self.collect_geom_info(model, spec)
        self.collect_mesh_path(spec)

    def save(self, save_path, max_faces=8000):
        """Save the parsed robot data using HumanoidRobot's save method."""
        self.humanoid_robot.save(save_path, mesh_path=self.mesh_path, max_faces=max_faces)

    def print_body_tree(self, colorful=False):
        pass
