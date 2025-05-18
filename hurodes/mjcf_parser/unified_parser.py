import os
import xml.etree.ElementTree as ET
import json
import shutil
from collections import defaultdict
from pathlib import Path

import pandas as pd
import numpy as np
import mujoco

from hurodes.contants import RobotFormatType
from hurodes.utils import is_int, is_float, get_elem_tree_str

def str2dict(string, name, dim_num=None):
    elements = string.split()
    if all([is_int(elem) for elem in elements]):
        data = np.array(elements, dtype=int)
    elif all([is_float(elem) for elem in elements]):
        data = np.array(elements, dtype=float)
    else:
        data = np.array(elements)
    return data2dict(data, name, dim_num)

def data2dict(data, name, dim_num=None):
    assert len(data.shape) == 1, f"Data shape should be 1D, but got {data.shape}"
    if dim_num is None:
        dim_num = data.shape[0]
    if dim_num == 1:
        return {name: data[0]}
    else:
        return {f"{name}{i}": data[i] for i in range(dim_num)}


class UnifiedMJCFParser:
    format_type = RobotFormatType.UNKNOWN

    def __init__(self, mjcf_path):
        self.mjcf_path = mjcf_path

        self.tree = ET.parse(self.mjcf_path)
        self.root = self.tree.getroot()

        self.worldbody = self.root.find("worldbody")
        assert self.worldbody, "No <worldbody> element found in the MJCF file."

        root_bodies = self.worldbody.findall("body")
        assert len(root_bodies) == 1, "There should be exactly one root <body> element in the <worldbody> element."
        self.base_link = root_bodies[0]

        self.body_parent_id = None
        self.mj_model_dict = None

        self.meshed_path = None
        self.mesh_file_type = None
        self.body_name2idx = None
        self.ground_dict = None

    def print_body_tree(self, colorful=False):
        print(get_elem_tree_str(self.base_link, colorful=colorful))

    def parse(self):
        self.parse_mujoco_model()
        self.parse_mujoco_xml()

    def parse_mujoco_model(self):
        self.mj_model_dict = defaultdict(list)
        spec = mujoco.MjSpec.from_file(self.mjcf_path)
        model = mujoco.MjModel.from_xml_path(self.mjcf_path)
        self.body_parent_id = model.body_parentid.tolist()
        self.body_name2idx = {}
        for body_idx in range(model.nbody):
            body = model.body(body_idx)
            self.body_name2idx[body.name] = body_idx
            body_dict = {"name": body.name}
            for key in ["pos", "quat", "inertia", "ipos", "iquat", "mass"]:
                body_dict |= data2dict(getattr(body, key), key)
            self.mj_model_dict["body"].append(body_dict)

        for jnt_idx in range(model.njnt):
            jnt = model.joint(jnt_idx)
            jnt_dict = {"name": jnt.name, "type": ["free", "ball", "slide", "hinge"][jnt.type[0]]}
            for key in ["pos", "axis", "range"]:
                jnt_dict |= data2dict(getattr(jnt, key), key)
            for key in ["armature", "damping", "frictionloss", "stiffness", "bodyid"]:
                jnt_dict |= data2dict(getattr(jnt, key), key, 1)
            self.mj_model_dict["joint"].append(jnt_dict)

        for geom_idx in range(model.ngeom):
            geom = model.geom(geom_idx)
            gtype = ["plane", "hfield","sphere", "capsule", "ellipsoid", "cylinder", "box", "mesh", "sdf"][geom.type[0]]
            if gtype in ["sphere", "capsule", "ellipsoid", "cylinder", "box"]:
                geom_dict = {"type": gtype}
                for key in ["pos", "quat", "size", "contype", "conaffinity", "bodyid", "friction"]:
                    geom_dict |= data2dict(getattr(geom, key), key)
                self.mj_model_dict["collision"].append(geom_dict)
            elif gtype == "mesh":
                pass # deal later
            elif gtype == "plane":
                assert geom.bodyid == 0, "Plane should be in worldbody."
                self.ground_dict = {
                    "contype": str(geom.contype[0]),
                    "conaffinity": str(geom.conaffinity[0]),
                    "friction": " ".join(map(str, geom.friction)),
                }
            else:
                raise NotImplementedError(f"Unsupported geom type: {gtype}")

        for actuator in spec.actuators:
            actuator_dict = {"name": actuator.name, "joint": actuator.target}
            actuator_dict |= data2dict(actuator.ctrlrange, "ctrlrange")
            self.mj_model_dict["actuator"].append(actuator_dict)


    def parse_mujoco_xml(self):
        self.meshed_path, self.mesh_file_type = {}, {}
        spec = mujoco.MjSpec.from_file(self.mjcf_path)
        meshdir = Path(spec.meshdir)
        for mesh in spec.meshes:
            mesh_file = (self.mjcf_path.parent / meshdir / mesh.file).resolve()
            mesh_type = mesh.file.split('.')[-1]
            self.meshed_path[mesh.name] = mesh_file
            self.mesh_file_type[mesh.name] = mesh_type

        for body in spec.bodies:
            idx = body.id
            for geom in body.geoms:
                if geom.type == mujoco.mjtGeom.mjGEOM_MESH:
                    mesh_dict = {"type": "mesh", "mesh": geom.meshname, "bodyid": idx}
                    mesh_dict |= data2dict(getattr(geom, 'pos', np.array([0,0,0])), "pos")
                    mesh_dict |= data2dict(getattr(geom, 'quat', np.array([1,0,0,0])), "quat")
                    mesh_dict |= data2dict(getattr(geom, 'rgba', np.array([0.5,0.5,0.5,1])), "rgba")
                    mesh_dict |= data2dict(getattr(geom, 'contype', np.array([1])), "contype")
                    mesh_dict |= data2dict(getattr(geom, 'conaffinity', np.array([1])), "conaffinity")
                    self.mj_model_dict["mesh"].append(mesh_dict)


    def save(self, save_path):
        os.makedirs(save_path, exist_ok=True)

        # copy mesh files
        os.makedirs(os.path.join(save_path, "meshes"), exist_ok=True)
        for name, path in self.meshed_path.items():
            shutil.copy(path, os.path.join(save_path, "meshes", f"{name}.{self.mesh_file_type[name]}"))

        # save meta info, including body tree
        meta_path = os.path.join(save_path, "meta.json")
        meta_info = {
            "format_type": self.format_type.value,
            "body_parent_id": self.body_parent_id,
            "mesh_file_type": self.mesh_file_type,
            "ground": self.ground_dict
        }
        with open(meta_path, "w") as json_file:
            json.dump(meta_info, json_file, indent=4)

        # save dict data
        for name, data_dict in self.mj_model_dict.items():
            df = pd.DataFrame(data_dict).sort_index(axis=1)
            df.to_csv(os.path.join(save_path, f"{name}.csv"), index=False)


if __name__ == "__main__":
    from hurodes import MJCF_ROBOTS_PATH, ROBOTS_PATH

    mjcf_path = os.path.join(MJCF_ROBOTS_PATH, "kuavo_s45", "mjcf", "biped_s45.xml")
    save_path = os.path.join(ROBOTS_PATH, "kuavo_s45")

    parser = UnifiedMJCFParser(mjcf_path)
    parser.print_body_tree()
    parser.parse()
    parser.save(save_path)
