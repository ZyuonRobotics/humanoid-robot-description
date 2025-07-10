from hurodes.utils.printing import get_elem_tree_str
import xml.etree.ElementTree as ET


def test_get_elem_tree_str_basic():
    # Create a simple XML tree
    root = ET.Element("body", name="root")
    child1 = ET.SubElement(root, "body", name="child1")
    child2 = ET.SubElement(root, "body", name="child2")
    grandchild = ET.SubElement(child1, "body", name="grandchild")
    tree_str = get_elem_tree_str(root, elem_tag="body", colorful=False)
    assert "root" in tree_str
    assert "child1" in tree_str
    assert "child2" in tree_str
    assert "grandchild" in tree_str

def test_get_elem_tree_str_colorful():
    root = ET.Element("body", name="root")
    ET.SubElement(root, "body", name="child")
    # Should not raise error with colorful=True
    tree_str = get_elem_tree_str(root, elem_tag="body", colorful=True)
    assert "root" in tree_str
    assert "child" in tree_str 