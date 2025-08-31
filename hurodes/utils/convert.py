from scipy.spatial.transform import Rotation

def quat2rpy(quat):
    qw, qx, qy, qz  = quat
    rot = Rotation.from_quat([qx, qy, qz, qw])
    euler = rot.as_euler("xyz", degrees=False)
    return euler

def str_quat2rpy(quat):
    rpy = quat2rpy(map(float, quat.split()))
    return " ".join([str(v) for v in rpy])

def rpy2quat(rpy):
    roll, pitch, yaw = rpy
    rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=False)
    qx, qy, qz, qw = rot.as_quat()
    return [qw, qx, qy, qz]

def str_rpy2quat(rpy):
    quat = rpy2quat(map(float, rpy.split()))
    return " ".join([str(v) for v in quat])