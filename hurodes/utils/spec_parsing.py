from collections import defaultdict
from pathlib import Path

import numpy as np
import mujoco

from hurodes.utils.typing import data2dict

def parse_mujoco_spec(spec):
    model_dict = defaultdict(list)
    body_name2idx = {}
    ground_dict = {}

    model = spec.compile()
    body_parent_id = model.body_parentid.tolist()
    for body_idx in range(model.nbody):
        body = model.body(body_idx)
        body_name2idx[body.name] = body_idx
        body_dict = {"name": body.name}
        for key in ["pos", "quat", "inertia", "ipos", "iquat", "mass"]:
            body_dict |= data2dict(getattr(body, key), key)
        model_dict["body"].append(body_dict)

    for jnt_idx in range(model.njnt):
        jnt = model.joint(jnt_idx)
        jnt_dict = {"name": jnt.name, "type": ["free", "ball", "slide", "hinge"][jnt.type[0]]}
        for key in ["pos", "axis", "range"]:
            jnt_dict |= data2dict(getattr(jnt, key), key)
        for key in ["armature", "damping", "frictionloss", "stiffness", "bodyid"]:
            jnt_dict |= data2dict(getattr(jnt, key), key, 1)
        model_dict["joint"].append(jnt_dict)

    for geom_idx in range(model.ngeom):
        geom = model.geom(geom_idx)
        gtype = ["plane", "hfield","sphere", "capsule", "ellipsoid", "cylinder", "box", "mesh", "sdf"][geom.type[0]]
        if gtype in ["sphere", "capsule", "ellipsoid", "cylinder", "box"]:
            geom_dict = {"type": gtype}
            for key in ["pos", "quat", "size", "contype", "conaffinity", "bodyid", "friction"]:
                geom_dict |= data2dict(getattr(geom, key), key)
            model_dict["collision"].append(geom_dict)
        elif gtype == "mesh":
            pass # deal later
        elif gtype == "plane":
            assert geom.bodyid == 0, "Plane should be in worldbody."
            ground_dict = {
                "contype": str(geom.contype[0]),
                "conaffinity": str(geom.conaffinity[0]),
                "friction": " ".join(map(str, geom.friction)),
            }
        else:
            raise NotImplementedError(f"Unsupported geom type: {gtype}")

    for actuator in spec.actuators:
        actuator_dict = {"name": actuator.name, "joint": actuator.target}
        actuator_dict |= data2dict(actuator.ctrlrange, "ctrlrange")
        model_dict["actuator"].append(actuator_dict)

    for body in spec.bodies:
        idx = body_name2idx[body.name]
        for geom in body.geoms:
            if geom.type == mujoco.mjtGeom.mjGEOM_MESH: # type: ignore
                mesh_dict = {"type": "mesh", "mesh": geom.meshname, "bodyid": idx}
                mesh_dict |= data2dict(getattr(geom, 'pos', np.array([0.0, 0.0, 0.0])), "pos")
                mesh_dict |= data2dict(getattr(geom, 'quat', np.array([1.0, 0.0, 0.0, 0.0])), "quat")
                mesh_dict |= data2dict(getattr(geom, 'rgba', np.array([0.5, 0.5, 0.5, 1.0])), "rgba")
                mesh_dict |= data2dict(getattr(geom, 'contype', np.array([1.0])), "contype")
                mesh_dict |= data2dict(getattr(geom, 'conaffinity', np.array([1.0])), "conaffinity")
                model_dict["mesh"].append(mesh_dict)
    model_dict["mesh"] = sorted(model_dict["mesh"], key=lambda x: x["bodyid"])

    return model_dict, ground_dict, body_parent_id


def get_mesh_dict(spec, file_path):
    mesh_path = {}
    mesh_file_type = {}

    meshdir = Path(spec.meshdir)
    if not meshdir.is_absolute():
        meshdir = (Path(file_path).parent / meshdir).resolve()
    assert meshdir.exists(), f"Mesh directory {meshdir} does not exist."

    for mesh in spec.meshes:
        mesh_path[mesh.name] = meshdir /  mesh.file
        mesh_file_type[mesh.name] = mesh.file.split('.')[-1]
    return mesh_path, mesh_file_type
