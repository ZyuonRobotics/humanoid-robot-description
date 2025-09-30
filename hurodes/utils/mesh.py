import shutil

MUJOCO_MIN_FACES = 1
MUJOCO_MAX_FACES = 200000



def simplify_obj(input_path, output_path, max_faces=8000):
    try: 
        import trimesh
    except ImportError:
        print("trimesh is not installed. Please install it with `pip install trimesh`")
        return
    
    assert MUJOCO_MIN_FACES <= max_faces <= MUJOCO_MAX_FACES, f"max_faces must be between {MUJOCO_MIN_FACES} and {MUJOCO_MAX_FACES}"

    mesh = trimesh.load(input_path)
    if isinstance(mesh, trimesh.Trimesh):
        original_faces = len(mesh.faces)
    else:
        raise TypeError(f"Loaded mesh is not a trimesh.Trimesh object: {type(mesh)}")
    
    if original_faces <= max_faces:
        shutil.copy(input_path, output_path)
        return
    
    # Calculate the ratio of faces to retain
    target_ratio = max_faces / original_faces
    
    # Apply quadric decimation to simplify the mesh
    # Note: trimesh uses (1 - target_ratio) as the decimation ratio
    simplified_mesh = mesh.simplify_quadric_decimation(1 - target_ratio)
    final_faces = len(simplified_mesh.faces)
    print(f"simplified mesh from {original_faces} to {final_faces} faces")
    
    simplified_mesh.export(output_path)
