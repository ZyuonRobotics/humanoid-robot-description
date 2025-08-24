import json
from collections import defaultdict
from multiprocessing.context import assert_spawning
from pathlib import Path
from abc import ABC

import pandas as pd
import mujoco

from hurodes.hrdf import ActuatorInfos, BodyInfos, SimpleGeomInfos, JointInfos, MeshInfos
from hurodes.utils.mesh import simplify_obj
from hurodes.hrdf.base.info import save_csv

PLANE_TYPE = int(mujoco.mjtGeom.mjGEOM_PLANE)
MESH_TYPE = int(mujoco.mjtGeom.mjGEOM_MESH)

def collect_body_info(model, spec):
    body_name2idx = {}
    body_infos_list = []
    
    assert model.body(0).name == "world", "First body should be world."
    for body_idx in range(1, model.nbody):
        body = model.body(body_idx)
        body_infos = BodyInfos.from_mujoco(body, spec.bodies[body_idx], model, spec)
        body_name2idx[body.name] = body_idx - 1
        body_infos_list.append(body_infos)
    return body_name2idx, body_infos_list

def collect_joint_info(model, spec):
    joint_infos_list = []

    assert model.joint(0).type[0] == 0, "First joint should be free."
    for jnt_idx in range(1, model.njnt):
        joint_infos = JointInfos.from_mujoco(model.joint(jnt_idx), spec.joints[jnt_idx], model, spec)
        joint_infos_list.append(joint_infos)
    return joint_infos_list

def collect_actuator_info(model, spec):
    actuator_infos_list = []
    for actuator_idx in range(model.nu):
        actuator_infos = ActuatorInfos.from_mujoco(model.actuator(actuator_idx), spec.actuators[actuator_idx], model, spec)
        actuator_infos_list.append(actuator_infos)
    return actuator_infos_list

def collect_geom_info(model, spec):
    mesh_infos_list, simple_geom_infos_list = [], []
    ground_dict = None
    for geom_idx in range(model.ngeom):
        geom_model, geom_spec = model.geom(geom_idx), spec.geoms[geom_idx]
        if int(geom_model.type) == MESH_TYPE:
            mesh_infos = MeshInfos.from_mujoco(geom_model, geom_spec, model, spec)
            mesh_infos_list.append(mesh_infos)
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
            geom_infos = SimpleGeomInfos.from_mujoco(geom_model, geom_spec, model, spec)
            simple_geom_infos_list.append(geom_infos)
    return mesh_infos_list, simple_geom_infos_list, ground_dict

def get_mesh_dict(spec, file_path):
    mesh_path = {}
    mesh_file_types = []

    meshdir = Path(spec.meshdir)
    if not meshdir.is_absolute():
        meshdir = (Path(file_path).parent / meshdir).resolve()
    assert meshdir.exists(), f"Mesh directory {meshdir} does not exist."

    for mesh in spec.meshes:
        mesh_path[mesh.name] = meshdir /  mesh.file
        mesh_file_types.append(mesh.file.split('.')[-1].lower())

    assert len(set(mesh_file_types)) == 1, "All mesh files must have the same file type."
    assert mesh_file_types[0] in ["obj", "stl"], "Mesh file type must be obj or stl."
    return mesh_path, mesh_file_types[0]

class BaseParser(ABC):
    def __init__(self, file_path):
        self.file_path = file_path
        
        self.body_infos_list = None
        self.joint_infos_list = None
        self.actuator_infos_list = None
        self.mesh_infos_list = None
        self.simple_geom_infos_list = None

        self.ground_dict = None
        self.body_parent_id = None

        self.mesh_path = {}
        self.mesh_file_type = None
        self.body_name2idx = {}


    @property
    def mujoco_spec(self):
        raise NotImplementedError("Subclasses must implement this method")

    def parse(self, base_link_name="base_link"):
        spec = self.mujoco_spec
        model = spec.compile()

        self.body_parent_id = (model.body_parentid[1:] - 1).tolist()

        self.body_name2idx, self.body_infos_list = collect_body_info(model, spec)
        self.joint_infos_list = collect_joint_info(model, spec)
        self.actuator_infos_list = collect_actuator_info(model, spec)
        self.mesh_infos_list, self.simple_geom_infos_list, self.ground_dict = collect_geom_info(model, spec)

        self.mesh_path, self.mesh_file_type = get_mesh_dict(spec, self.file_path)

    def save(self, save_path, max_faces=8000):
        save_path = Path(save_path)
        save_path.mkdir(parents=True, exist_ok=True)
        for name, path in self.mesh_path.items():
            new_mesh_file = save_path / "meshes" / f"{name}.{self.mesh_file_type}"
            new_mesh_file.parent.mkdir(parents=True, exist_ok=True)
            simplify_obj(path, new_mesh_file, max_faces)
        
        meta_path = save_path / "meta.json"
        meta_path.touch(exist_ok=True)
        meta_info = {
            "body_parent_id": self.body_parent_id,
            "mesh_file_type": self.mesh_file_type,
            "ground": self.ground_dict
        }
        with open(meta_path, "w") as json_file:
            json.dump(meta_info, json_file, indent=4)
        
        for infos_name in ["body", "joint", "actuator", "mesh", "simple_geom"]:
            infos_list = getattr(self, f"{infos_name}_infos_list")
            save_csv(infos_list, save_path / f"{infos_name}.csv")

    def print_body_tree(self, colorful=False):
        pass
