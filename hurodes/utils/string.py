from colorama import Fore, Style
from typing import Optional, Tuple, Dict
import re

def get_elem_tree_str(elem, indent=0, elem_tag="body", colorful=False):
    res = ""

    colors = [Fore.BLUE, Fore.GREEN, Fore.YELLOW, Fore.RED, Fore.MAGENTA, Fore.CYAN]
    color = colors[indent % len(colors)]
    indent_symbols = "  " * indent
    if colorful:
        res += color + indent_symbols + Style.RESET_ALL
    else:
        res += indent_symbols

    name = elem.get("name", "unnamed")
    if colorful:
        res += Fore.WHITE + name + "\n"
    else:
        res += name + "\n"

    for child in elem.findall(elem_tag):
        res += get_elem_tree_str(child, indent + 1, colorful=colorful)
    return res

def get_prefix_name(prefix, name):
    if name == "" or name is None:
        return name
    elif prefix is None:
        return name
    else:
        return f"{prefix}_{name}"
        
def filter_str_list(str_list: list[str], pos_strings: list[str] = None, neg_strings: list[str] = None):
    if pos_strings is None:
        pos_strings = []
    if neg_strings is None:
        neg_strings = []
    
    res = []
    for string in str_list:
        if all(neg_s not in string for neg_s in neg_strings) and all(pos_s in string for pos_s in pos_strings):
            res.append(string)
    return res

def parse_inertia_file(content: str) -> Optional[Tuple[str, float, Dict[str, float]]]:
    coord_system_match = re.search(r'坐标系：\s*(\w+)', content)
    assert coord_system_match, "cannot extract coord_system"
    
    link_name = coord_system_match.group(1)
    
    # Extract mass (supports scientific notation)
    mass_match = re.search(r'质量\s*=\s*([\d.]+(?:[eE][-+]?\d+)?)\s*千克', content)
    assert mass_match, "cannot extract mass"
    
    mass = float(mass_match.group(1))
    
    # Extract inertia tensor
    inertia_section = re.search(
        r'由重心决定，并且对齐输出的坐标系.*?\n(.*?Lxx.*?\n.*?Lyx.*?\n.*?Lzx.*?\n)',
        content, re.DOTALL
    )
    
    assert inertia_section, "cannot extract inertia tensor"
    
    inertia_text = inertia_section.group(1)
    
    # Parse inertia tensor values
    inertia_dict = {}
    
    # Extract inertia tensor components (supports scientific notation and negative numbers)
    patterns = {
        'ixx': r'Lxx\s*=\s*([-+]?[\d.]+(?:[eE][-+]?\d+)?)',
        'ixy': r'Lxy\s*=\s*([-+]?[\d.]+(?:[eE][-+]?\d+)?)',
        'ixz': r'Lxz\s*=\s*([-+]?[\d.]+(?:[eE][-+]?\d+)?)',
        'iyy': r'Lyy\s*=\s*([-+]?[\d.]+(?:[eE][-+]?\d+)?)',
        'iyz': r'Lyz\s*=\s*([-+]?[\d.]+(?:[eE][-+]?\d+)?)',
        'izz': r'Lzz\s*=\s*([-+]?[\d.]+(?:[eE][-+]?\d+)?)'
    }
    
    for key, pattern in patterns.items():
        match = re.search(pattern, inertia_text)
        assert match, f"cannot extract inertia tensor component {key}"
        inertia_dict[key] = float(match.group(1))
        if key in ["ixy", "ixz", "iyz"]:
            inertia_dict[key] = -inertia_dict[key]
    
    return link_name, mass, inertia_dict
