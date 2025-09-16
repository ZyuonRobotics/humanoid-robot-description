import pytest
import numpy as np
from dataclasses import dataclass
from typing import Union, Type

from hurodes.hrdf.base.info import InfoBase
from hurodes.hrdf.base.attribute import AttributeBase, Name, Position, Id


# Create simple test attributes for testing
@dataclass
class SampleName(Name):
    """Simple test name attribute"""
    name: str = "test_name"
    mujoco_name: str = "test_name"
    urdf_path: tuple = ("name",)


@dataclass
class SamplePosition(Position):
    """Simple test position attribute"""
    name: str = "test_pos"
    dim: int = 3
    mujoco_name: str = "test_pos"
    urdf_path: tuple = ("origin", "xyz")


@dataclass
class SampleId(Id):
    """Simple test ID attribute"""
    name: str = "test_id"
    dtype: Union[Type, str] = int
    mujoco_name: str = "test_id"


@dataclass
class SampleValue(AttributeBase):
    """Simple test scalar value attribute"""
    name: str = "test_value"
    dtype: Union[Type, str] = float
    dim: int = 0
    mujoco_name: str = "test_value"
    urdf_path: tuple = ("value",)


# Create a test info class that uses these attributes
class SampleInfo(InfoBase):
    """Test info class containing various attribute types"""
    info_name: str = "TestInfo"
    attr_classes: tuple = (SampleName, SamplePosition, SampleId, SampleValue)


@pytest.fixture
def test_data():
    """Test fixture providing sample test data"""
    return {
        "test_name": "test_robot",
        "test_pos": [1.0, 2.0, 3.0],
        "test_id": 42,
        "test_value": 3.14
    }


@pytest.fixture
def test_flat_data():
    """Test fixture providing sample flat test data"""
    return {
        "test_name": "test_robot",
        "test_pos0": 1.0,
        "test_pos1": 2.0,
        "test_pos2": 3.0,
        "test_id": 42,
        "test_value": 3.14
    }


def test_info_creation_from_dict(test_data):
    """Test creating info object from dictionary"""
    info = SampleInfo.from_dict(test_data)
    
    # Test that info object was created successfully
    assert isinstance(info, SampleInfo)
    assert info.info_name == "TestInfo"
    
    # Test attribute access
    assert info["test_name"].data == "test_robot"
    assert info["test_id"].data == 42
    assert info["test_value"].data == 3.14
    
    # Test position array
    np.testing.assert_array_equal(info["test_pos"].data, np.array([1.0, 2.0, 3.0]))


def test_info_creation_from_flat_dict(test_flat_data):
    """Test creating info object from flat dictionary"""
    info = SampleInfo.from_flat_dict(test_flat_data)
    
    # Test that info object was created successfully
    assert isinstance(info, SampleInfo)
    
    # Test attribute access
    assert info["test_name"].data == "test_robot"
    assert info["test_id"].data == 42
    assert info["test_value"].data == 3.14
    
    # Test position array
    np.testing.assert_array_equal(info["test_pos"].data, np.array([1.0, 2.0, 3.0]))


def test_to_dict_conversion(test_data):
    """Test converting info object to dictionary"""
    info = SampleInfo.from_dict(test_data)
    result_dict = info.to_flat_dict()
    
    # Test that all expected keys are present
    expected_keys = {"test_name", "test_pos0", "test_pos1", "test_pos2", "test_id", "test_value"}
    assert set(result_dict.keys()) == expected_keys
    
    # Test values
    assert result_dict["test_name"] == "test_robot"
    assert result_dict["test_id"] == 42
    assert result_dict["test_value"] == 3.14
    assert result_dict["test_pos0"] == 1.0
    assert result_dict["test_pos1"] == 2.0
    assert result_dict["test_pos2"] == 3.0


def test_attribute_access(test_data):
    """Test accessing attributes by name"""
    info = SampleInfo.from_dict(test_data)
    
    # Test valid attribute access
    name_attr = info["test_name"]
    assert isinstance(name_attr, SampleName)
    assert name_attr.data == "test_robot"
    
    pos_attr = info["test_pos"]
    assert isinstance(pos_attr, SamplePosition)
    np.testing.assert_array_equal(pos_attr.data, np.array([1.0, 2.0, 3.0]))
    
    # Test invalid attribute access
    with pytest.raises(AssertionError, match="Attribute non_existent_attr not found"):
        _ = info["non_existent_attr"]


def test_mujoco_dict_generation(test_data):
    """Test generating Mujoco dictionary from info object"""
    info = SampleInfo.from_dict(test_data)
    mujoco_dict = info.to_mujoco_dict()
    
    # Test that mujoco names are used
    expected_keys = {"test_name", "test_pos", "test_id", "test_value"}
    assert set(mujoco_dict.keys()) == expected_keys
    
    # Test values
    assert mujoco_dict["test_name"] == "test_robot"
    assert mujoco_dict["test_id"] == "42"
    assert mujoco_dict["test_value"] == "3.14"
    assert mujoco_dict["test_pos"] == "1.0 2.0 3.0"


def test_info_with_none_values():
    """Test info creation with None values"""
    data_with_none = {
        "test_name": None,
        "test_pos": None,
        "test_id": None,
        "test_value": None
    }
    
    info = SampleInfo.from_dict(data_with_none)
    
    # Test that info object was created successfully
    assert isinstance(info, SampleInfo)
    
    # Test that None values are handled properly
    assert info["test_name"].data is None
    assert info["test_pos"].data is None
    assert info["test_id"].data is None
    assert info["test_value"].data is None


def test_round_trip_conversion(test_data):
    """Test converting info to dict and back preserves data"""
    original_info = SampleInfo.from_dict(test_data)
    flat_dict = original_info.to_flat_dict()
    reconstructed_info = SampleInfo.from_flat_dict(flat_dict)
    
    # Test that data is preserved
    assert original_info["test_name"].data == reconstructed_info["test_name"].data
    assert original_info["test_id"].data == reconstructed_info["test_id"].data
    assert original_info["test_value"].data == reconstructed_info["test_value"].data
    np.testing.assert_array_equal(original_info["test_pos"].data, reconstructed_info["test_pos"].data)


def test_info_repr(test_data):
    """Test string representation of info object"""
    info = SampleInfo.from_dict(test_data)
    repr_str = repr(info)
    
    # Test that repr contains expected information
    assert "TestInfo" in repr_str
    assert "test_name" in repr_str
    assert "test_pos" in repr_str
    assert "test_id" in repr_str
    assert "test_value" in repr_str
