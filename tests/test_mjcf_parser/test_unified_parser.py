import numpy as np
import pytest
from hurodes.mjcf_parser.unified_parser import str2dict, data2dict, UnifiedMJCFParser

def test_str2dict_int():
    s = "1 2 3"
    d = str2dict(s, "a")
    assert isinstance(d, dict)
    assert d["a0"] == 1 and d["a1"] == 2 and d["a2"] == 3

def test_str2dict_float():
    s = "1.1 2.2 3.3"
    d = str2dict(s, "b")
    assert isinstance(d, dict)
    assert d["b0"] == 1.1 and d["b1"] == 2.2 and d["b2"] == 3.3

def test_str2dict_str():
    s = "foo bar baz"
    d = str2dict(s, "c")
    assert isinstance(d, dict)
    assert d["c0"] == "foo" and d["c1"] == "bar" and d["c2"] == "baz"

def test_data2dict_scalar():
    assert data2dict(5, "x") == {"x": 5}
    assert data2dict(3.14, "y") == {"y": 3.14}

def test_data2dict_array():
    arr = np.array([1, 2, 3])
    d = data2dict(arr, "z")
    assert d == {"z0": 1, "z1": 2, "z2": 3}

def test_unified_mjcf_parser_init(tmp_path):
    # Create a minimal MJCF XML file for testing
    xml_content = """
    <mujoco>
        <worldbody>
            <body name='base'>
            </body>
        </worldbody>
    </mujoco>
    """
    xml_path = tmp_path / "test.xml"
    xml_path.write_text(xml_content)
    parser = UnifiedMJCFParser(str(xml_path))
    assert parser.mjcf_path == str(xml_path)
    assert parser.worldbody is not None
    assert parser.base_link is not None 