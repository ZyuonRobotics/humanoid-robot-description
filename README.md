# Humanoid Robot Description
Transfer MJCF to EHDF (Exclusive Humanoid Description Format).

## Installation

```bash
conda create -n hurodes python=3.10
conda activate hurodes
pip install -e .
``` 

## Usage

### MJCF to EHDF

You are able to convert MJCF to EHDF by running the following command:

```bash
python scripts/mjcf_to_ehdf.py --input_path assets/mjcf_robots/ --robot_name your_robot_name
```

EHDF of your robot will be saved in `assets/robots/your_robot_name`, as the following structure:
```
assets/robots/your_robot_name/
├── actuator.csv
├── body.csv
├── collision.csv
├── joint.csv
├── mesh.csv
├── meshes
│   ├── base_link.STL
│   ├── ...
└── meta.json
```

### URDF to EHDF

To convert URDF to EHDF, you first need to convert URDF to MJCF format. You can use the provided `urdf2mjcf.py` script which automates this process.

The script performs the following operations:
- Automatically adds required MuJoCo elements to the URDF file (mujoco compiler settings, dummy link, and floating joint)
- Optimizes mesh files by reducing face count to improve simulation performance
- Converts the modified URDF to MJCF format using MuJoCo's built-in converter
- Saves the output MJCF file in the same directory as the input URDF

Usage:
```bash
python scripts/urdf2mjcf.py --urdf_path path/to/your/robot.urdf --robot_name your_robot_name --keep_percent 8000
```

Parameters:
- `--urdf_path`: Path to the input URDF file
- `--robot_name`: Name for the output MJCF file
- `--keep_percent`: Target number of faces for mesh optimization (default: 8000)

The script automatically handles:
- Mesh optimization to reduce computational load
- Adding MuJoCo compiler directives with correct mesh directory paths
- Creating dummy link and floating joint for proper robot base configuration

After conversion, you can then convert the MJCF to EHDF using the standard MJCF to EHDF workflow.

### EHDF to MJCF

Save the EHDF file in `assets/robots/your_robot_name`, then run the following command:
```bash
python scripts/ehdf2mjcf.py --robot_name your_robot_name
```