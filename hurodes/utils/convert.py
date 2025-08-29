from scipy.spatial.transform import Rotation

def str_quat2rpy(quat):
    qw, qx, qy, qz = quat.split()
    rot = Rotation.from_quat([qx, qy, qz, qw])
    euler = rot.as_euler("xyz", degrees=False)
    return " ".join([str(e) for e in euler])