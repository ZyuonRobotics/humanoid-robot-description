import xml.etree.ElementTree as ET


def add_attr_to_elem(elem, attr_path, attr_value):
    if len(attr_path) == 1:
        if isinstance(attr_path[0], str):
            elem.set(attr_path[0], str(attr_value))
        elif isinstance(attr_path[0], tuple):
            values = attr_value.split()
            assert len(values) == len(attr_path[0]), f"Expected {len(attr_path[0])} values for {attr_path[0]}, got {len(values)}"
            for i, value in enumerate(values):
                elem.set(attr_path[0][i], value)
    else:
        sub_elem_list = elem.findall(attr_path[0])
        if len(sub_elem_list) == 0:
            sub_elem = ET.SubElement(elem, attr_path[0])
        elif len(sub_elem_list) == 1:
            sub_elem = sub_elem_list[0]
        else:
            raise ValueError(f"Found multiple elements with tag {attr_path[0]}")
        add_attr_to_elem(sub_elem, attr_path[1:], attr_value)

def extract_attr_from_elem(elem, attr_path):
    if len(attr_path) == 1:
        if isinstance(attr_path[0], str):
            return elem.get(attr_path[0])
        elif isinstance(attr_path[0], tuple):
            values = []
            for attr_name in attr_path[0]:
                values.append(elem.get(attr_name))
            return values
        else:
            raise ValueError(f"Invalid attribute path: {attr_path[0]}")
    else:
        sub_elem = elem.findall(attr_path[0])
        if sub_elem is None or len(sub_elem) == 0:
            return None
        elif len(sub_elem) == 1:
            return extract_attr_from_elem(sub_elem[0], attr_path[1:])
        else:
            raise ValueError(f"Found multiple elements with tag {attr_path[0]}")
