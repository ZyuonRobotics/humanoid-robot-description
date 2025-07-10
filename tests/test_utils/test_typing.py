from hurodes.utils.typing import is_int, is_float, str2dict, data2dict, dict2str
import numpy as np

def test_is_int():
    assert is_int("123")
    assert is_int("-456")
    assert not is_int("1.23")
    assert not is_int("abc")

def test_is_float():
    assert is_float("1.23")
    assert is_float("-4.56")
    assert is_float("123")  # int is also float
    assert not is_float("abc")

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

def test_dict2str():
    data = {
        "a0": 1, "a1": 2, "a2": 3,
        "b": 1.1,
        "c0": 1.1, "c1": 1.2, "c2": 1.3
    }
    assert dict2str(data, "a") == "1 2 3"
    assert dict2str(data, "b") == "1.1"
    assert dict2str(data, "c") == "1.1 1.2 1.3"