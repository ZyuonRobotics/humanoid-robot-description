import pytest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import Mock, patch
import tempfile
import os

from hurodes.generators.generator_base import GeneratorBase


class MockGenerator(GeneratorBase):
    """Mock implementation of GeneratorBase for testing concrete methods"""
    
    def __init__(self):
        super().__init__()
        self._mock_xml_root = None
        
    @property
    def xml_root(self) -> ET.Element:
        """Create a mock XML root element for testing"""
        if self._mock_xml_root is None:
            self._mock_xml_root = ET.Element("mock_root")
        return self._mock_xml_root
    
    def load(self):
        """Mock implementation of load method"""
        # Mock load method - just mark as loaded
        pass
    
    def generate(self, prefix=None):
        """Mock implementation of generate method"""
        # Add a test element to the XML root
        test_elem = ET.SubElement(self.xml_root, "test_element")
        if prefix:
            test_elem.set("prefix", prefix)


@pytest.fixture
def mock_generator():
    """Fixture providing a MockGenerator instance"""
    return MockGenerator()


@pytest.fixture
def temp_file():
    """Fixture providing a temporary file for export testing"""
    with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.xml') as f:
        temp_path = Path(f.name)
    yield temp_path
    # Cleanup
    if temp_path.exists():
        temp_path.unlink()


class TestGeneratorBase:
    """Test suite for GeneratorBase class"""
    
    def test_initialization(self, mock_generator):
        """Test that GeneratorBase initializes correctly"""
        assert mock_generator._xml_root is None
        assert not mock_generator._loaded
    
    def test_xml_root_property(self, mock_generator):
        """Test xml_root property creates and returns XML element"""
        root = mock_generator.xml_root
        assert isinstance(root, ET.Element)
        assert root.tag == "mock_root"
        
        # Test that subsequent calls return the same element
        root2 = mock_generator.xml_root
        assert root is root2
    
    def test_destroy(self, mock_generator):
        """Test destroy method resets XML root"""
        # First create an XML root
        _ = mock_generator.xml_root
        assert mock_generator._mock_xml_root is not None
        
        # Destroy and verify it's reset
        mock_generator.destroy()
        assert mock_generator._xml_root is None
    