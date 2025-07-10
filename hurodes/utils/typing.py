import numpy as np

def is_int(s):
    try:
        int(s)
        return True
    except ValueError:
        return False

def is_float(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

def str2dict(string, name, dim_num=None):
    """
    Convert a string to a dictionary.
    """
    elements = string.split()
    if all([is_int(elem) for elem in elements]):
        data = np.array(elements, dtype=int)
    elif all([is_float(elem) for elem in elements]):
        data = np.array(elements, dtype=float)
    else:
        data = np.array(elements)
    return data2dict(data, name, dim_num)

def data2dict(data, name, dim_num=None):
    """
    Convert a numpy array to a dictionary.
    If the array is a scalar, return a dictionary with the key name and the value as the scalar.
    If the dim_num is not None, use only first dim_num data, or use all data, return a dictionary with the name and index as the key, and corresponding data as the value.
    """
    if type(data) == int or type(data) == float:
        return {name: data}
    assert len(data.shape) == 1, f"Data shape should be 1D, but got {data.shape}"
    if dim_num is None:
        dim_num = data.shape[0]
    if dim_num == 1:
        return {name: data[0]}
    else:
        return {f"{name}{i}": data[i] for i in range(dim_num)}

def dict2str(data, name):
    """
    Convert a dictionary to a string.
    If the dictionary has only one key starting with name, return the value of the key.
    If the dictionary has multiple keys starting with name, return a space-separated string of the values.
    """
    keys_in_dict = [key for key in data.keys() if key.startswith(name)]
    assert len(keys_in_dict) > 0, f"No key starts with {name} in data: {data}"
    if len(keys_in_dict) == 1:
        assert keys_in_dict[0] == name, f"keys_in_dict: {keys_in_dict}"
        return str(data[name])
    else:
        for i in range(len(keys_in_dict)):
            assert f"{name}{i}" in keys_in_dict, f"{name}{i} not in keys_in_dict: {keys_in_dict}"
        return " ".join([str(data[f"{name}{i}"]) for i in range(len(keys_in_dict))])