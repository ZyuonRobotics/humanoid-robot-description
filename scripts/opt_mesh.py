import os
import click
import shutil

import trimesh
from tqdm import tqdm

"""
dependence:
- trimesh
- fast_simplification
"""


def simplify_obj(input_path, output_path, percent=0.8):
    mesh = trimesh.load(input_path)
    simplified_mesh = mesh.simplify_quadric_decimation(percent=percent)
    simplified_mesh.export(output_path)

@click.command()
@click.option("--mesh_path", prompt='mesh path', type=str)
@click.option("--percent", type=float, default=0.8)
def main(mesh_path, percent):
    # backup all mesh in path
    assert os.path.isdir(mesh_path)
    os.makedirs(mesh_path + "_backup", exist_ok=True)
    for name in tqdm(os.listdir(mesh_path)):
        obj_path = os.path.join(mesh_path, name)
        backup_obj_path = os.path.join(mesh_path + "_backup", name)
        shutil.copy(obj_path, backup_obj_path)

        simplify_obj(obj_path, obj_path, percent)

if __name__ == "__main__":
    main()
