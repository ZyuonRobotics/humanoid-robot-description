import click
import trimesh
import shutil
import mujoco
from pathlib import Path
from tqdm import tqdm
from typing import Union
import xml.etree.ElementTree as ET

MUJOCO_MIN_FACES = 1
MUJOCO_MAX_FACES = 200000

def get_optim_meshes_path(urdf_path: Union[str, Path]) -> Path:
    """
    Get the path for optimized meshes based on the URDF file path.
    
    Args:
        urdf_path (str or Path): Path to the URDF file.
    
    Returns:
        optim_meshes_path (Path): Path to the optimized meshes directory.
    """
    urdf_path = Path(urdf_path)
    optim_meshes_path = urdf_path.parent.parent / "optimized_meshes"
    
    if not optim_meshes_path.exists():
        optim_meshes_path.mkdir(parents=True, exist_ok=True)
    
    return optim_meshes_path

def get_meshes_path(urdf_path: Union[str, Path], meshes_path = None) -> Path:
    """
    Get the meshes path based on the URDF file path.
    
    Args:
        urdf_path (str or Path): Path to the URDF file.
        meshes_path (str or Path, optional): Custom meshes path. If not provided, it defaults to the parent directory of the URDF file.
    
    Returns:
        meshes_path (Path): Path to the meshes directory.
    """
    if meshes_path is None:
        urdf_path = Path(urdf_path)
        meshes_path = urdf_path.parent.parent / "meshes"
    else:
        meshes_path = Path(meshes_path)
    
    if not meshes_path.exists():
        raise FileNotFoundError(f"Meshes path does not exist: {meshes_path}")
    
    return meshes_path

def get_meshes(meshes_path):
    """
    Get all mesh files from the meshes directory.
    
    Args:
        meshes_path (str or Path): Path to the meshes directory
        
    Yields:
        Path: Path objects for each mesh file found
    """
    return filter(lambda f: f.suffix.lower() in ['.stl', '.obj'], Path(meshes_path).glob('*'))


def check_meshes(urdf_path, meshes_path=None) -> bool:
    """
    Check if the URDF project contains meshes. Just check if there is a meshes path
    Args:
        urdf_path (str): Path to the URDF file.
    Returns:
        bool: True if the URDF contains meshes, False otherwise.
    """
    meshes_path = get_meshes_path(urdf_path, meshes_path)
    if meshes_path.exists() and meshes_path.is_dir():
        print(f"Mesh path found: {meshes_path}")
        print(f"The number of meshes: {len(list(get_meshes(meshes_path)))}")
        return True
    return False

def simplify_obj(input_path, output_path, max_faces=8000):
    """
    Simplify a 3D mesh by reducing the number of faces using quadric decimation.
    
    Args:
        input_path (str or Path): Path to the input mesh file
        output_path (str or Path): Path to save the simplified mesh
        max_faces (int): Target number of faces for the simplified mesh
    """
    # Load the mesh from the input file
    mesh = trimesh.load(input_path)
    if isinstance(mesh, trimesh.Trimesh):
        original_faces = len(mesh.faces)
    else:
        raise TypeError(f"Loaded mesh is not a trimesh.Trimesh object: {type(mesh)}")
    print(f"Original faces: {original_faces}")
    
    # Check if simplification is needed
    if original_faces <= max_faces:
        print(f"Mesh already has {original_faces} faces (≤ {max_faces}), no simplification needed")
        # If the face count is already less than or equal to the target value, directly copy the original file (if input and output paths are different)
        shutil.copy(input_path, output_path)
        return
    
    # Calculate the ratio of faces to retain
    target_ratio = max_faces / original_faces
    
    # Apply quadric decimation to simplify the mesh
    # Note: trimesh uses (1 - target_ratio) as the decimation ratio
    simplified_mesh = mesh.simplify_quadric_decimation(1 - target_ratio)
    final_faces = len(simplified_mesh.faces)
    print(f"Simplified faces: {final_faces}")
    
    # Export the simplified mesh to the output path
    simplified_mesh.export(output_path)

def optimize_meshes_faces(urdf_path, meshes_path, max_faces=8000):
    """
    Optimize all mesh files in the URDF project by reducing their face count.
    
    This function processes all mesh files in the meshes directory, simplifies them
    to reduce computational load, and saves the optimized versions to a separate directory.
    
    Args:
        urdf_path (str or Path): Path to the URDF file
        meshes_path (str or Path): Path to the meshes directory
        max_faces (int): Target number of faces for each optimized mesh
    """
    # Get the source meshes directory path
    meshes_path = get_meshes_path(urdf_path, meshes_path)
    
    # Get the destination directory for optimized meshes
    optimize_meshes_path = get_optim_meshes_path(urdf_path)
    
    # Create the optimized meshes directory if it doesn't exist
    if not optimize_meshes_path.exists():
        optimize_meshes_path.mkdir(parents=True, exist_ok=True)
    
    # Get all mesh files
    meshes = list(get_meshes(meshes_path))
    
    # Process all mesh files with a progress bar
    with tqdm(total=len(meshes), desc="Optimizing meshes") as pbar:
        for mesh_file in meshes:
            # Update progress bar description with current file name
            pbar.set_description(f"Optimizing {mesh_file.name}")
            
            # Define the output path for the optimized mesh
            optimize_mesh_file = optimize_meshes_path / mesh_file.name
            
            # Simplify the current mesh file
            simplify_obj(mesh_file, optimize_mesh_file, max_faces)
            
            # Update the progress bar
            pbar.update(1)


@click.command()
@click.option("--urdf_path", prompt='URDF path', type=str, help="Path to the input URDF file (xml).")
@click.option("--max_faces", type=int, default=8000, help="Max faces to keep in the mesh optimization.")
@click.option("--meshes_path", type=str, default=None, help="Path to the meshes directory. If not provided, defaults to the parent directory of the URDF file.")
@click.option("--default_actuator", type=bool, default=True)
def main(urdf_path, max_faces, meshes_path, default_actuator):
    """
    Convert a URDF file to MJCF format and save it in the specified directory.
    """
    assert check_meshes(urdf_path, meshes_path), "The URDF project does not contain meshes. Please check again."

    urdf_root = ET.parse(urdf_path).getroot()
    
    # Ensure the root element is a robot tag
    if urdf_root.tag != 'robot':
        print("Warning: Root element is not 'robot'")
        return
    
    # Create mujoco tag
    mujoco_elem = ET.Element('mujoco')
    optim_meshes_path = str(get_optim_meshes_path(urdf_path).resolve())
    ET.SubElement(mujoco_elem, 'compiler', {'meshdir': optim_meshes_path, 'balanceinertia': 'true', 'discardvisual': 'false'})
    
    # Create dummy_link
    dummy_link = ET.Element('link', {'name': 'dummy_link'})
    
    # Create floating joint
    dummy_joint = ET.Element('joint', {'name': 'dummy_to_base_link', 'type': 'floating'})
    
    # Add origin
    ET.SubElement(dummy_joint, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
    
    # Add parent and child
    ET.SubElement(dummy_joint, 'parent', {'link': 'dummy_link'})
    ET.SubElement(dummy_joint, 'child', {'link': 'base_link'})
    
    # Insert new elements at the beginning of the robot tag
    urdf_root.insert(0, mujoco_elem)
    urdf_root.insert(1, dummy_link)
    urdf_root.insert(2, dummy_joint)
    
    optimize_meshes_faces(urdf_path, meshes_path, max_faces)

    urdf_string = ET.tostring(urdf_root, encoding='utf-8', xml_declaration=True).decode('utf-8')
    mjspec = mujoco.MjSpec.from_string(urdf_string) # type: ignore
    mjcf_string = mjspec.to_xml()

    if default_actuator:
        mjcf_root = ET.fromstring(mjcf_string)
        actuator_elem = ET.SubElement(mjcf_root, 'actuator')
        for joint_spec in mjspec.joints:
            if int(joint_spec.type) == 3: # only deal with hinges
                motor_elem = ET.SubElement(
                    actuator_elem, 'motor',
                    name=f"{joint_spec.name}_motor",
                    joint=joint_spec.name
                )
        tree = ET.ElementTree(mjcf_root)
        ET.indent(tree, space="  ", level=0)
        mjcf_string = ET.tostring(mjcf_root, encoding='utf-8', xml_declaration=True).decode('utf-8')

    output_path = Path(urdf_path).parent / f"{Path(urdf_path).stem}.xml"
    output_path.write_text(mjcf_string, encoding='utf-8')


if __name__ == "__main__":
    main()