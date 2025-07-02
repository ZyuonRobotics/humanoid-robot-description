import pytest
from hurodes import utils
import xml.etree.ElementTree as ET

def test_is_int():
    assert utils.is_int("123")
    assert utils.is_int("-456")
    assert not utils.is_int("1.23")
    assert not utils.is_int("abc")

def test_is_float():
    assert utils.is_float("1.23")
    assert utils.is_float("-4.56")
    assert utils.is_float("123")  # int is also float
    assert not utils.is_float("abc")

def test_get_elem_tree_str_basic():
    # Create a simple XML tree
    root = ET.Element("body", name="root")
    child1 = ET.SubElement(root, "body", name="child1")
    child2 = ET.SubElement(root, "body", name="child2")
    grandchild = ET.SubElement(child1, "body", name="grandchild")
    tree_str = utils.get_elem_tree_str(root, elem_tag="body", colorful=False)
    assert "root" in tree_str
    assert "child1" in tree_str
    assert "child2" in tree_str
    assert "grandchild" in tree_str

def test_get_elem_tree_str_colorful():
    root = ET.Element("body", name="root")
    ET.SubElement(root, "body", name="child")
    # Should not raise error with colorful=True
    tree_str = utils.get_elem_tree_str(root, elem_tag="body", colorful=True)
    assert "root" in tree_str
    assert "child" in tree_str 