import pytest
import numpy as np
import pandas as pd
import tempfile
import os

from hurodes.hrdf.base.info import InfoList
from hurodes.hrdf.infos.body import BodyInfo

INFO_DICT = {
        "mass": 2.5,
        "diag_inertia": [0.5, 0.6, 0.7],
        "pos": [1.0, 2.0, 3.0],
        "inertial_pos": [0.1, 0.2, 0.3],
        "quat": [1.0, 0.0, 0.0, 0.0],
        "inertial_quat": [0.5, 0.5, 0.5, 0.5],
        "name": "body_1",
        "id": 1
    }
    
INFO_DICT2 = {
    "mass": 1.8,
    "diag_inertia": [0.2, 0.3, 0.4],
    "pos": [-1.0, -2.0, 3.0],
    "inertial_pos": [-0.1, 0.2, -0.3],
    "quat": [0.0, 1.0, 0.0, 0.0],
    "inertial_quat": [0.707, 0.0, 0.707, 0.0],
    "name": "body_2_negative_test",
    "id": 2
}

INFO_DICT3 = {
    "mass": 0.001,
    "diag_inertia": [1e-6, 1e-5, 1e-4],
    "pos": [0.001, 0.002, 0.003],
    "inertial_pos": [1e-8, 1e-7, 1e-6],
    "quat": [1.0, 1e-10, 1e-10, 1e-10],
    "inertial_quat": [0.999999, 0.000001, 0.0, 0.0],
    "name": "micro_body",
    "id": 999
}

def test_body_info_comprehensive():
    """Test various data scenarios for BodyInfo"""
    body_info = BodyInfo.from_dict(INFO_DICT)
    
    # Test basic data types
    assert body_info["mass"].data == INFO_DICT["mass"]
    assert body_info["name"].data == INFO_DICT["name"]
    assert body_info["id"].data == INFO_DICT["id"]
    
    # Test array data
    np.testing.assert_array_equal(body_info["diag_inertia"].data, INFO_DICT["diag_inertia"])
    np.testing.assert_array_equal(body_info["pos"].data, INFO_DICT["pos"])
    np.testing.assert_array_equal(body_info["inertial_pos"].data, INFO_DICT["inertial_pos"])
    np.testing.assert_array_equal(body_info["quat"].data, INFO_DICT["quat"])
    np.testing.assert_array_equal(body_info["inertial_quat"].data, INFO_DICT["inertial_quat"])
    
    # Test conversion to flat dictionary
    flat_dict = body_info.to_flat_dict()
    assert flat_dict["mass"] == INFO_DICT["mass"]
    assert flat_dict["name"] == INFO_DICT["name"]
    assert flat_dict["id"] == INFO_DICT["id"]
    assert flat_dict["diag_inertia0"] == INFO_DICT["diag_inertia"][0]
    assert flat_dict["diag_inertia1"] == INFO_DICT["diag_inertia"][1]
    assert flat_dict["diag_inertia2"] == INFO_DICT["diag_inertia"][2]
    assert flat_dict["pos0"] == INFO_DICT["pos"][0]
    assert flat_dict["pos1"] == INFO_DICT["pos"][1]
    assert flat_dict["pos2"] == INFO_DICT["pos"][2]
    
    # Test recreating from flat dictionary
    body_info2 = BodyInfo.from_flat_dict(flat_dict)
    assert body_info2["mass"].data == INFO_DICT["mass"]
    assert body_info2["name"].data == INFO_DICT["name"]
    np.testing.assert_array_equal(body_info2["diag_inertia"].data, INFO_DICT["diag_inertia"])


def test_body_info_csv_operations():
    """Test CSV save and load operations for BodyInfo"""
    info_list = InfoList(BodyInfo)

    for info_dict in [INFO_DICT, INFO_DICT2, INFO_DICT3]:
        info_list.append(BodyInfo.from_dict(info_dict))
    
    # Save to temporary CSV file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False) as f:
        csv_path = f.name
    
    try:
        # Save CSV
        info_list.save_csv(csv_path)
        
        # Load CSV
        loaded_info_list = InfoList.from_csv(csv_path, BodyInfo)
        
        # Verify data integrity
        assert len(loaded_info_list) == 3
        
        # Verify first object
        assert loaded_info_list[0]["mass"].data == INFO_DICT["mass"]
        assert loaded_info_list[0]["name"].data == INFO_DICT["name"]
        assert loaded_info_list[0]["id"].data == INFO_DICT["id"]
        
        # Verify second object's negative numbers
        assert loaded_info_list[1]["name"].data == INFO_DICT2["name"]
        np.testing.assert_array_equal(loaded_info_list[1]["pos"].data, INFO_DICT2["pos"])
        
        # Verify third object's very small values
        assert loaded_info_list[2]["mass"].data == INFO_DICT3["mass"]
        assert loaded_info_list[2]["id"].data == INFO_DICT3["id"]
        assert abs(loaded_info_list[2]["diag_inertia"].data[0] - 1e-6) < 1e-10
        
    finally:
        os.unlink(csv_path)


def test_body_info_mujoco_generation():
    """Test Mujoco dictionary generation for BodyInfo"""
    info_dict = {
        "mass": 3.0,
        "diag_inertia": [0.8, 0.9, 1.0],
        "pos": [2.0, 3.0, 4.0],
        "inertial_pos": [0.2, 0.3, 0.4],
        "quat": [0.5, 0.5, 0.5, 0.5],
        "inertial_quat": [0.866, 0.0, 0.0, 0.5],
        "name": "mujoco_test_body",
        "id": 10
    }
    
    body_info = BodyInfo.from_dict(info_dict)
    
    # Test Mujoco dictionary generation for body tag
    mujoco_dict_body = body_info.to_mujoco_dict(tag="body")
    assert "name" in mujoco_dict_body
    assert "pos" in mujoco_dict_body
    assert "quat" in mujoco_dict_body
    assert mujoco_dict_body["name"] == "mujoco_test_body"
    assert mujoco_dict_body["pos"] == "2.0 3.0 4.0"
    
    # Test Mujoco dictionary generation for inertial tag
    mujoco_dict_inertial = body_info.to_mujoco_dict(tag="inertial")
    assert "mass" in mujoco_dict_inertial
    assert "diaginertia" in mujoco_dict_inertial
    assert mujoco_dict_inertial["mass"] == "3.0"
    assert mujoco_dict_inertial["diaginertia"] == "0.8 0.9 1.0"


def test_body_info_error_cases():
    """Test error cases for BodyInfo"""
    body_info = BodyInfo.from_dict({
        "mass": 1.0,
        "name": "test",
        "id": 1,
        "diag_inertia": [1, 1, 1],
        "pos": [0, 0, 0],
        "inertial_pos": [0, 0, 0],
        "quat": [1, 0, 0, 0],
        "inertial_quat": [1, 0, 0, 0]
    })
    
    # Test invalid tag
    with pytest.raises(ValueError, match="Invalid tag"):
        body_info.to_mujoco_dict(tag="invalid_tag")
    
    # Test accessing non-existent attribute
    with pytest.raises(AssertionError, match="not found in info"):
        body_info["nonexistent_attribute"]

