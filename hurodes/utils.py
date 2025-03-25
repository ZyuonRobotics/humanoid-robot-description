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

if __name__ == '__main__':
    for s in ["11", "-11", "1.1", "ss"]:
        print(f"{s} is int: {is_int(s)}")
        print(f"{s} is float: {is_float(s)}")
