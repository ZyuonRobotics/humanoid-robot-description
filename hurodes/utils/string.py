from colorama import Fore, Style

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
        