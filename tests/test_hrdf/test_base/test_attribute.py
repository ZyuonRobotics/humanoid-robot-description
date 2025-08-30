import pytest
import numpy as np
from dataclasses import dataclass
from typing import Union, Type

from hurodes.hrdf.base.attribute import AttributeBase, TYPE_MAP


# Test fixtures for sample data
@pytest.fixture
def sample_scalar_data():
    """Fixture providing scalar test data"""
    return {
        'int_value': 42,
        'float_value': 3.14,
        'str_value': "test_string",
        'bool_value': True
    }


@pytest.fixture
def sample_array_data():
    """Fixture providing array test data"""
    return {
        'position': [1.0, 2.0, 3.0],
        'quaternion': [0.0, 0.0, 0.0, 1.0],
        'axis': [1.0, 0.0, 0.0]
    }


@pytest.fixture
def sample_flat_dict():
    """Fixture providing flat dictionary test data"""
    return {
        'name': "test_name",
        'pos0': 1.0,
        'pos1': 2.0,
        'pos2': 3.0,
        'quat0': 0.0,
        'quat1': 0.0,
        'quat2': 0.0,
        'quat3': 1.0,
        'axis0': 1.0,
        'axis1': 0.0,
        'axis2': 0.0,
        'id': 123
    }


# Test custom attribute classes for testing
@dataclass
class SampleScalarAttribute(AttributeBase):
    """Test scalar attribute for unit testing"""
    name: str = "test_scalar"
    dtype: Union[Type, str] = float
    dim: int = 0
    mujoco_name: str = "test_scalar"
    urdf_path: tuple = ("scalar",)


@dataclass
class SampleVectorAttribute(AttributeBase):
    """Test vector attribute for unit testing"""
    name: str = "test_vector"
    dtype: Union[Type, str] = float
    dim: int = 3
    mujoco_name: str = "test_vector"
    urdf_path: tuple = ("vector", "xyz")


@dataclass
class SampleStringAttribute(AttributeBase):
    """Test string attribute for unit testing"""
    name: str = "test_string"
    dtype: Union[Type, str] = str
    dim: int = 0
    mujoco_name: str = "test_string"


class TestAttributeBase:
    """Test the AttributeBase class functionality"""

    def test_scalar_attribute_creation(self):
        """Test creating a scalar attribute"""
        attr = SampleScalarAttribute()
        assert attr.name == "test_scalar"
        assert attr.dtype == float
        assert attr.dim == 0
        assert attr.mujoco_name == "test_scalar"
        assert attr.urdf_path == ("scalar",)
        assert attr.data is None
    
    def test_vector_attribute_creation(self):
        """Test creating a vector attribute"""
        attr = SampleVectorAttribute()
        assert attr.name == "test_vector"
        assert attr.dtype == float
        assert attr.dim == 3
        assert attr.mujoco_name == "test_vector"
        assert attr.urdf_path == ("vector", "xyz")
        assert attr.data is None
    
    def test_scalar_data_assignment(self, sample_scalar_data):
        """Test assigning scalar data to attributes"""
        # Test float assignment
        attr = SampleScalarAttribute()
        attr.data = sample_scalar_data['float_value']
        assert attr.data == 3.14
        assert isinstance(attr.data, float)
        
        # Test int assignment (should be converted to float)
        attr.data = sample_scalar_data['int_value']
        assert attr.data == 42.0
        assert isinstance(attr.data, float)
    
    def test_vector_data_assignment(self, sample_array_data):
        """Test assigning vector data to attributes"""
        attr = SampleVectorAttribute()
        
        # Test list assignment
        attr.data = sample_array_data['position']
        np.testing.assert_array_equal(attr.data, np.array([1.0, 2.0, 3.0]))
        assert attr.data.dtype == float
        
        # Test numpy array assignment
        arr = np.array([4.0, 5.0, 6.0])
        attr.data = arr
        np.testing.assert_array_equal(attr.data, arr)
    
    def test_none_data_assignment(self):
        """Test assigning None to data"""
        attr = SampleScalarAttribute()
        attr.data = None
        assert attr.data is None
        
        vector_attr = SampleVectorAttribute()
        vector_attr.data = None
        assert vector_attr.data is None
    
    def test_invalid_vector_dimension(self):
        """Test invalid vector dimension assignment"""
        attr = SampleVectorAttribute()  # expects dim=3
        
        with pytest.raises(AssertionError, match="Invalid data shape"):
            attr.data = [1.0, 2.0]  # Wrong dimension
        
        with pytest.raises(AssertionError, match="Invalid data shape"):
            attr.data = [1.0, 2.0, 3.0, 4.0]  # Wrong dimension
    
    def test_invalid_vector_type(self):
        """Test invalid vector type assignment"""
        attr = SampleVectorAttribute()
        
        with pytest.raises(AssertionError, match="Invalid data type"):
            attr.data = "invalid_string"  # String instead of array
    
    def test_parse_flat_dict_scalar(self):
        """Test parsing flat dictionary for scalar attributes"""
        attr = SampleScalarAttribute()
        flat_dict = {'test_scalar': 42.5}
        attr.parse_flat_dict(flat_dict)
        assert attr.data == 42.5
    
    def test_parse_flat_dict_vector(self):
        """Test parsing flat dictionary for vector attributes"""
        attr = SampleVectorAttribute()
        flat_dict = {
            'test_vector0': 1.0,
            'test_vector1': 2.0,
            'test_vector2': 3.0
        }
        attr.parse_flat_dict(flat_dict)
        np.testing.assert_array_equal(attr.data, np.array([1.0, 2.0, 3.0]))
    
    def test_parse_flat_dict_missing_key(self):
        """Test parsing flat dictionary with missing keys"""
        attr = SampleScalarAttribute()
        flat_dict = {}
        
        with pytest.raises(AssertionError, match="Attribute test_scalar not found"):
            attr.parse_flat_dict(flat_dict)
    
    def test_parse_flat_dict_missing_vector_key(self):
        """Test parsing flat dictionary with missing vector keys"""
        attr = SampleVectorAttribute()
        flat_dict = {
            'test_vector0': 1.0,
            'test_vector1': 2.0
            # Missing test_vector2
        }
        
        with pytest.raises(AssertionError, match="Attribute test_vector2 not found"):
            attr.parse_flat_dict(flat_dict)
    
    def test_from_flat_dict_classmethod(self, sample_flat_dict):
        """Test creating attribute from flat dictionary using classmethod"""
        attr = SampleVectorAttribute.from_flat_dict({
            'test_vector0': 1.0,
            'test_vector1': 2.0,
            'test_vector2': 3.0
        })
        np.testing.assert_array_equal(attr.data, np.array([1.0, 2.0, 3.0]))
    
    def test_from_data_classmethod(self, sample_array_data):
        """Test creating attribute from data using classmethod"""
        attr = SampleVectorAttribute.from_data(sample_array_data['position'])
        np.testing.assert_array_equal(attr.data, np.array([1.0, 2.0, 3.0]))
    
    def test_to_dict_scalar(self):
        """Test converting scalar attribute to dictionary"""
        attr = SampleScalarAttribute.from_data(42.5)
        result = attr.to_dict()
        assert result == {'test_scalar': 42.5}
    
    def test_to_dict_vector(self):
        """Test converting vector attribute to dictionary"""
        attr = SampleVectorAttribute.from_data([1.0, 2.0, 3.0])
        result = attr.to_dict()
        expected = {'test_vector0': 1.0, 'test_vector1': 2.0, 'test_vector2': 3.0}
        assert result == expected
    
    def test_to_dict_none_scalar(self):
        """Test converting None scalar attribute to dictionary"""
        attr = SampleScalarAttribute()
        result = attr.to_dict()
        assert result == {'test_scalar': None}
    
    def test_to_dict_none_vector(self):
        """Test converting None vector attribute to dictionary"""
        attr = SampleVectorAttribute()
        result = attr.to_dict()
        expected = {'test_vector0': None, 'test_vector1': None, 'test_vector2': None}
        assert result == expected
    
    def test_to_string_scalar(self):
        """Test converting scalar attribute to string"""
        attr = SampleScalarAttribute.from_data(42.5)
        assert attr.to_string() == "42.5"
    
    def test_to_string_vector(self):
        """Test converting vector attribute to string"""
        attr = SampleVectorAttribute.from_data([1.0, 2.0, 3.0])
        assert attr.to_string() == "1.0 2.0 3.0"
    
    def test_to_string_string_nan(self):
        """Test converting string attribute with 'nan' value"""
        attr = SampleStringAttribute.from_data("nan")
        assert attr.to_string() == ""  # Special case for "nan" strings
    
    def test_to_string_regular_string(self):
        """Test converting regular string attribute"""
        attr = SampleStringAttribute.from_data("test_value")
        assert attr.to_string() == "test_value"
