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

you need to transfer URDF to MJCF first, then convert MJCF to EHDF. Add an extension to original URDF file like

```xml

<robot name="darwin">
    <mujoco>
        <compiler meshdir="../mesh/darwin/" balanceinertia="true" discardvisual="false"/>
    </mujoco>
    <link name="dummy_link"/>
    <joint name="dummy_to_base_link" type="floating">
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <parent link="dummy_link"/>
        <child link="base_link"/>
    </joint>
    ...
</robot>
```

Make sure these issues are checked:
- mesh files are not too large: use scripts/opt_mesh.py to reduce faces
- mujoco element: to tell the compiler where to find the mesh files
- dummy link and joint: to make the robot a floating base

Then run mojoco by following command, and drag the URDF file to mujoco window to see the robot, and save the MJCF file by click "Save xml" button.
```bash
python -m mujoco.viewer
```

### EHDF to MJCF

Save the EHDF file in `assets/robots/your_robot_name`, then run the following command:
```bash
python scripts/ehdf2mjcf.py --robot_name your_robot_name
```