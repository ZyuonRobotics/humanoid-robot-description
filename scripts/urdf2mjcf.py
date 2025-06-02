import click
import trimesh
import shutil
import mujoco
from pathlib import Path
import xml.etree.ElementTree as ET
from tqdm import tqdm

from hurodes import MJCF_ROBOTS_PATH, ROBOTS_PATH


MESHES_PATH = lambda x: Path(x).parent.parent / "meshes"
OPTIM_MESHES_PATH = lambda x: Path(x).parent.parent / "optimized_meshes"
MUJOCO_MIN_FACES = 1
MUJOCO_MAX_FACES = 200000

def check_meshes(urdf_path) -> bool:
    """
    Check if the URDF project contains meshes. Just check if there is a meshes path
    Args:
        urdf_path (str): Path to the URDF file.
    Returns:
        bool: True if the URDF contains meshes, False otherwise.
    """
    mesh_path = MESHES_PATH(urdf_path)
    if mesh_path.exists() and mesh_path.is_dir():
        print(f"Mesh path found: {mesh_path}")
        print(f"The number of meshes: {len(list(mesh_path.glob('*.STL')))}")
        return True
    return False

def simplify_obj(input_path, output_path, target_faces=8000):
    """
    Simplify a 3D mesh by reducing the number of faces using quadric decimation.
    
    Args:
        input_path (str or Path): Path to the input mesh file
        output_path (str or Path): Path to save the simplified mesh
        target_faces (int): Target number of faces for the simplified mesh
    """
    # Load the mesh from the input file
    mesh = trimesh.load(input_path)
    original_faces = len(mesh.faces)
    print(f"Original faces: {original_faces}")
    
    # Check if simplification is needed
    if original_faces <= target_faces:
        print(f"Mesh already has {original_faces} faces (≤ {target_faces}), no simplification needed")
        # If the face count is already less than or equal to the target value, directly copy the original file (if input and output paths are different)
        shutil.copy(input_path, output_path)
        return
    
    # Calculate the ratio of faces to retain
    target_ratio = target_faces / original_faces
    
    # Apply quadric decimation to simplify the mesh
    # Note: trimesh uses (1 - target_ratio) as the decimation ratio
    simplified_mesh = mesh.simplify_quadric_decimation(1 - target_ratio)
    final_faces = len(simplified_mesh.faces)
    print(f"Simplified faces: {final_faces}")
    
    # Export the simplified mesh to the output path
    simplified_mesh.export(output_path)

def optimize_meshes_faces(urdf_path, target_faces=8000):
    """
    Optimize all STL mesh files in the URDF project by reducing their face count.
    
    This function processes all .STL files in the meshes directory, simplifies them
    to reduce computational load, and saves the optimized versions to a separate directory.
    
    Args:
        urdf_path (str or Path): Path to the URDF file
        target_faces (int): Target number of faces for each optimized mesh
    """
    # Get the source meshes directory path
    meshes = MESHES_PATH(urdf_path)
    
    # Get the destination directory for optimized meshes
    optimize_meshes_path = OPTIM_MESHES_PATH(urdf_path)
    
    # Create the optimized meshes directory if it doesn't exist
    if not optimize_meshes_path.exists():
        optimize_meshes_path.mkdir(parents=True, exist_ok=True)
    
    # Process all STL files with a progress bar
    with tqdm(total=len(list(meshes.glob('*.STL'))), desc="Optimizing meshes") as pbar:
        for mesh_file in meshes.glob('*.STL'):
            # Update progress bar description with current file name
            pbar.set_description(f"Optimizing {mesh_file.name}")
            
            # Define the output path for the optimized mesh
            # backup_mesh_file = backup_meshes / mesh_file.name  # Commented backup option
            optimize_mesh_file = optimize_meshes_path / mesh_file.name
            
            # Simplify the current mesh file
            simplify_obj(mesh_file, optimize_mesh_file, target_faces)
            
            # Update the progress bar
            pbar.update(1)


@click.command()
@click.option("--urdf_path", prompt='URDF path', type=str, help="Path to the input URDF file (xml).")
@click.option("--keep_percent", type=int, default=500, help="Percentage of faces to keep in the mesh optimization.")
@click.option("--robot_name", prompt='Robot name', type=str, help="Name of the robot.")
def main(urdf_path, keep_percent, robot_name):
    """
    Convert a URDF file to MJCF format and save it in the specified directory.
    """
    assert check_meshes(urdf_path), "The URDF project does not contain meshes. Please check again."

    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    # Ensure the root element is a robot tag
    if root.tag != 'robot':
        print("Warning: Root element is not 'robot'")
        return
    
    # Create mujoco tag
    mujoco_elem = ET.Element('mujoco')
    compiler_elem = ET.SubElement(mujoco_elem, 'compiler')
    compiler_elem.set('meshdir', str(OPTIM_MESHES_PATH(urdf_path).resolve()))
    compiler_elem.set('balanceinertia', 'true')
    compiler_elem.set('discardvisual', 'false')
    
    # Create dummy_link
    dummy_link = ET.Element('link')
    dummy_link.set('name', 'dummy_link')
    
    # Create floating joint
    dummy_joint = ET.Element('joint')
    dummy_joint.set('name', 'dummy_to_base_link')
    dummy_joint.set('type', 'floating')
    
    # Add origin
    origin_elem = ET.SubElement(dummy_joint, 'origin')
    origin_elem.set('xyz', '0 0 0')
    origin_elem.set('rpy', '0 0 0')
    
    # Add parent and child
    parent_elem = ET.SubElement(dummy_joint, 'parent')
    parent_elem.set('link', 'dummy_link')
    
    child_elem = ET.SubElement(dummy_joint, 'child')
    child_elem.set('link', 'base_link')
    
    # Insert new elements at the beginning of the robot tag
    root.insert(0, mujoco_elem)
    root.insert(1, dummy_link)
    root.insert(2, dummy_joint)
    
    optimize_meshes_faces(urdf_path, keep_percent)

    xml_string = ET.tostring(root, encoding='utf-8', xml_declaration=True).decode('utf-8')
    mjspec = mujoco.MjSpec.from_string(xml_string)
    output_path = Path(urdf_path).parent / f"{robot_name}_mjcf.xml"
    output_path.write_text(mjspec.to_xml(), encoding='utf-8')


if __name__ == "__main__":
    main()