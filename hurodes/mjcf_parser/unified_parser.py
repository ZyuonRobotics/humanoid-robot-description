import os
import xml.etree.ElementTree as ET
import json
from collections import defaultdict
import shutil

from colorama import Fore, Style
import pandas as pd
import numpy as np
import mujoco

from hurodes.contants import RobotFormatType
from hurodes.utils import is_int, is_float

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

    def print_body_tree(self, body_element=None, indent=0):
        if body_element is None:
            body_element = self.base_link

        colors = [Fore.BLUE, Fore.GREEN, Fore.YELLOW, Fore.RED, Fore.MAGENTA, Fore.CYAN]
        color = colors[indent % len(colors)]
        indent_symbols = "│ " * indent + "├─ "
        colored_indent = color + indent_symbols + Style.RESET_ALL

        name = body_element.get("name", "unnamed")
        print(colored_indent + Fore.WHITE + name)

        for child in body_element.findall("body"):
            self.print_body_tree(child, indent + 1)

    def parse(self):
        self.parse_mujoco_model()
        self.parse_mujoco_xml()

    def parse_mujoco_model(self):
        self.mj_model_dict = defaultdict(list)

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

        for actuator_idx in range(model.nu):
            actuator = model.actuator(actuator_idx)
            joint = self.mj_model_dict["joint"][actuator.trnid[0]]["name"]
            actuator_dict = {"name": actuator.name, "joint": joint}
            actuator_dict |= data2dict(actuator.ctrlrange, "ctrlrange")
            self.mj_model_dict["actuator"].append(actuator_dict)


    def parse_mujoco_xml(self):
        self.meshed_path, self.mesh_file_type = {}, {}
        if "meshdir" in self.root.find("compiler").attrib:
            meshdir = self.root.find("compiler").attrib["meshdir"]
            meshdir = os.path.join(os.path.dirname(self.mjcf_path), meshdir)
        else:
            meshdir = os.path.dirname(self.mjcf_path)
        for mesh_elem in self.root.find("asset").findall("mesh"):
            self.meshed_path[mesh_elem.get("name")] = os.path.join(meshdir, mesh_elem.get("file"))
            self.mesh_file_type[mesh_elem.get("name")] = mesh_elem.get("file").split(".")[-1]

        for body_elem in self.root.findall(".//body"):
            body_idx = self.body_name2idx[body_elem.get("name")]
            for geom_elem in body_elem.findall("geom"):
                if geom_elem.get("type") == "mesh":
                    mesh_dict = {"type": "mesh", "mesh": geom_elem.get("mesh"), "bodyid": body_idx}
                    mesh_dict |= str2dict(geom_elem.get("pos", "0 0 0"), "pos")
                    mesh_dict |= str2dict(geom_elem.get("quat", "1 0 0 0"), "quat")
                    mesh_dict |= str2dict(geom_elem.get("rgba", "0.5 0.5 0.5 1"), "rgba")
                    mesh_dict |= str2dict(geom_elem.get("contype", "1"), "contype")
                    mesh_dict |= str2dict(geom_elem.get("conaffinity", "1"), "conaffinity")
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
