from pathlib import Path

import pytest
import numpy as np
import mujoco
import trimesh

from hurodes.utils.spec_parsing import parse_mujoco_spec, get_mesh_dict


# Test XML content
TEST_XML_CONTENT = '''<?xml version="1.0" encoding="UTF-8"?>
<mujoco model="test_robot">
    <compiler meshdir="meshes"/>
    <worldbody>
        <geom type="plane" pos="0 0 0" size="0 0 1" contype="1" conaffinity="1"/>
        <body name="base" pos="0 0 1">
            <joint name="base_joint" type="free"/>
            <geom type="box" pos="0 0 0" size="0.1 0.1 0.1"/>
            <body name="link1" pos="0 0 1.2">
                <joint name="joint1" type="hinge" axis="0 0 1" range="-3.14 3.14"/>
                <geom type="capsule" pos="0 0 0.1" size="0.05 0.1"/>
                <body name="link2" pos="0 0 1.2">
                    <joint name="joint2" type="slide" axis="0 1 0" range="-0.5 0.5"/>
                    <geom type="sphere" pos="0 0 0.2" size="0.1"/>
                    <body name="link3" pos="0 0 1.2">
                        <joint name="joint3" type="ball" axis="0 0 1"/>
                        <geom type="cylinder" pos="0 0 0.2" size="0.05 0.2"/>
                        <geom type="mesh" mesh="test_mesh" pos="0 0 0.4" rgba="1 0 0 1"/>
                    </body>
                </body>
            </body>
        </body>
    </worldbody>

    <actuator>
        <motor name="motor1" joint="joint1" ctrlrange="-10 10"/>
        <motor name="motor2" joint="joint2" ctrlrange="-5 5"/>
    </actuator>

    <asset>
        <mesh name="test_mesh" file="test_mesh.stl"/>
    </asset>
</mujoco>
'''


@pytest.fixture
def test_xml_file(tmp_path):
    """Create test XML file"""
    # Create mesh directory and files
    mesh_dir = tmp_path / "meshes"
    mesh_dir.mkdir()
    mesh_file = mesh_dir / "test_mesh.stl"
    # generate a real mesh file, using trimesh
    mesh = trimesh.creation.box()
    mesh.export(mesh_file)
    
    # Create XML file
    xml_file = tmp_path / "test_robot.xml"
    xml_file.write_text(TEST_XML_CONTENT)
    return xml_file


@pytest.fixture
def test_mesh_dir(tmp_path):
    """Create test mesh directory and files"""
    mesh_dir = tmp_path / "meshes"
    mesh_dir.mkdir()
    mesh_file = mesh_dir / "test_mesh.stl"
    mesh_file.write_text("dummy mesh content")
    return mesh_dir


def test_parse_mujoco_spec_basic(test_xml_file):
    """Test basic parsing functionality"""
    spec = mujoco.MjSpec.from_file(str(test_xml_file)) # type: ignore
    model_dict, ground_dict, body_parent_id = parse_mujoco_spec(spec)
    
    # Check returned dictionary structure
    assert isinstance(model_dict, dict)
    assert isinstance(ground_dict, dict)
    assert isinstance(body_parent_id, list)
    
    # Check if expected keys are included
    expected_keys = ["body", "joint", "collision", "mesh", "actuator"]
    for key in expected_keys:
        assert key in model_dict


def test_parse_mujoco_spec_body(test_xml_file):
    """Test body parsing functionality"""
    spec = mujoco.MjSpec.from_file(str(test_xml_file)) # type: ignore
    model_dict, _, _ = parse_mujoco_spec(spec)
    
    bodies = model_dict["body"]
    assert len(bodies) == 5  # world, base, link1, link2, link3
    
    # Check body names
    body_names = [body["name"] for body in bodies]
    expected_names = ["world", "base", "link1", "link2", "link3"]
    assert body_names == expected_names
    
    # Check body properties
    for body in bodies:
        assert "name" in body
        assert "mass" in body
        for name, dim in zip(["pos", "quat", "inertia"], [3, 4, 3]):
            for i in range(dim):
                assert f"{name}{i}" in body


def test_parse_mujoco_spec_joint(test_xml_file):
    """Test joint parsing functionality"""
    spec = mujoco.MjSpec.from_file(str(test_xml_file)) # type: ignore
    model_dict, _, _ = parse_mujoco_spec(spec)
    
    joints = model_dict["joint"]
    assert len(joints) == 4  # base_joint, joint1, joint2, joint3
    
    # Check joint names and types
    joint_info = {joint["name"]: joint["type"] for joint in joints}
    expected_joints = {
        "base_joint": "free",
        "joint1": "hinge", 
        "joint2": "slide",
        "joint3": "ball"
    }
    assert joint_info == expected_joints
    
    # Check joint properties
    for joint in joints:
        assert "name" in joint
        assert "type" in joint
        assert "armature" in joint
        assert "damping" in joint
        assert "frictionloss" in joint
        for name, dim in zip(["pos", "axis", "range"], [3, 3, 2]):
            for i in range(dim):
                assert f"{name}{i}" in joint


def test_parse_mujoco_spec_collision(test_xml_file):
    """Test collision geometry parsing functionality"""
    spec = mujoco.MjSpec.from_file(str(test_xml_file)) # type: ignore
    model_dict, _, _ = parse_mujoco_spec(spec)
    
    collisions = model_dict["collision"]
    assert len(collisions) == 4  # box, capsule, sphere, cylinder
    
    # Check geometry types
    geom_types = [collision["type"] for collision in collisions]
    expected_types = ["box", "capsule", "sphere", "cylinder"]
    assert geom_types == expected_types
    
    # Check geometry properties
    for collision in collisions:
        assert "type" in collision
        assert "contype" in collision
        assert "conaffinity" in collision
        assert "bodyid" in collision
        for name, dim in zip(["pos", "size"], [3, 3]):
            for i in range(dim):
                assert f"{name}{i}" in collision



def test_parse_mujoco_spec_mesh(test_xml_file):
    """Test mesh parsing functionality"""
    spec = mujoco.MjSpec.from_file(str(test_xml_file)) # type: ignore
    model_dict, _, _ = parse_mujoco_spec(spec)
    
    meshes = model_dict["mesh"]
    assert len(meshes) == 1  # Only one mesh
    
    mesh = meshes[0]
    assert mesh["type"] == "mesh"
    assert mesh["mesh"] == "test_mesh"
    assert "bodyid" in mesh
    assert "contype" in mesh
    assert "conaffinity" in mesh
    for name, dim in zip(["pos", "quat", "rgba"], [3, 4, 4]):
        for i in range(dim):
            assert f"{name}{i}" in mesh



def test_parse_mujoco_spec_actuator(test_xml_file):
    """Test actuator parsing functionality"""
    spec = mujoco.MjSpec.from_file(str(test_xml_file)) # type: ignore
    model_dict, _, _ = parse_mujoco_spec(spec)
    
    actuators = model_dict["actuator"]
    assert len(actuators) == 2  # motor1, motor2
    
    # Check actuator information
    actuator_info = {act["name"]: act["joint"] for act in actuators}
    expected_actuators = {
        "motor1": "joint1",
        "motor2": "joint2"
    }
    assert actuator_info == expected_actuators
    
    # Check actuator properties
    for actuator in actuators:
        assert "name" in actuator
        assert "joint" in actuator
        assert "ctrlrange0" in actuator
        assert "ctrlrange1" in actuator


def test_parse_mujoco_spec_ground(test_xml_file):
    """Test ground plane parsing functionality"""
    spec = mujoco.MjSpec.from_file(str(test_xml_file)) # type: ignore
    _, ground_dict, _ = parse_mujoco_spec(spec)
    
    # Check ground plane properties
    assert "contype" in ground_dict
    assert "conaffinity" in ground_dict
    assert "friction" in ground_dict
    
    # Check value types
    assert isinstance(ground_dict["contype"], str)
    assert isinstance(ground_dict["conaffinity"], str)
    assert isinstance(ground_dict["friction"], str)


def test_parse_mujoco_spec_body_parent_id(test_xml_file):
    """Test body parent id parsing functionality"""
    spec = mujoco.MjSpec.from_file(str(test_xml_file)) # type: ignore
    _, _, body_parent_id = parse_mujoco_spec(spec)
    
    # Check body_parent_id
    assert isinstance(body_parent_id, list)
    assert len(body_parent_id) == 5  # 5 bodies
    
    # Check parent id values (0 represents worldbody)
    assert body_parent_id[0] == 0  # base's parent is worldbody
    assert body_parent_id[1] == 0  # link1's parent is base
    assert body_parent_id[2] == 1  # link2's parent is link1
    assert body_parent_id[3] == 2  # link3's parent is link2
