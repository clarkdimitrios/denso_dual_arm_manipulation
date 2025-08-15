import numpy as np
import xml.etree.ElementTree as ET

from math import radians, degrees
import transforms3d
from geometry_msgs.msg import Pose, Quaternion

def get_xacro_properties(xacro_path):
    """
    Parses a Xacro file directly to extract xacro:property tags.
    Does not evaluate the file; just extracts scalar values.
    """
    ns = {'xacro': 'http://www.ros.org/wiki/xacro'}
    tree = ET.parse(xacro_path)
    root = tree.getroot()

    props = {}
    for prop in root.findall(".//xacro:property", ns):
        name = prop.attrib.get("name")
        value = prop.attrib.get("value")
        if name and value:
            props[name] = value
    return props



def pose_euler_to_quat(x, y, z, roll, pitch, yaw):
    "Convert Euler angles (deg) to quaternions and returns Pose"
    quat = euler_to_quat(roll, pitch, yaw)
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = quat[1]
    pose.orientation.y = quat[2]
    pose.orientation.z = quat[3]
    pose.orientation.w = quat[0]
    return pose

def euler_to_quat(roll, pitch, yaw):
    "Convert Euler angles (deg) to quaternions"
    return transforms3d.euler.euler2quat(
        radians(roll), radians(pitch), radians(yaw), axes='sxyz'
    )

def quat_to_euler_rad(q):
    if isinstance(q, Quaternion):
        q = [q.w, q.x, q.y, q.z]
    return transforms3d.euler.quat2euler(q, axes='sxyz')

def quat_to_euler_deg(q):
    if isinstance(q, Quaternion):
        q = [q.w, q.x, q.y, q.z]
    return [degrees(angle) for angle in transforms3d.euler.quat2euler(q, axes='sxyz')]

def rot_to_quat_xyzw(R: np.ndarray) -> Quaternion:
    m = R
    t = np.trace(m)
    if t > 0:
        S = np.sqrt(t + 1.0) * 2
        w = 0.25 * S
        x = (m[2,1] - m[1,2]) / S
        y = (m[0,2] - m[2,0]) / S
        z = (m[1,0] - m[0,1]) / S
    else:
        i = int(np.argmax([m[0,0], m[1,1], m[2,2]]))
        if i == 0:
            S = np.sqrt(1.0 + m[0,0] - m[1,1] - m[2,2]) * 2
            x = 0.25 * S
            y = (m[0,1] + m[1,0]) / S
            z = (m[0,2] + m[2,0]) / S
            w = (m[2,1] - m[1,2]) / S
        elif i == 1:
            S = np.sqrt(1.0 + m[1,1] - m[0,0] - m[2,2]) * 2
            y = 0.25 * S
            x = (m[0,1] + m[1,0]) / S
            z = (m[1,2] + m[2,1]) / S
            w = (m[0,2] - m[2,0]) / S
        else:
            S = np.sqrt(1.0 + m[2,2] - m[0,0] - m[1,1]) * 2
            z = 0.25 * S
            x = (m[0,2] + m[2,0]) / S
            y = (m[1,2] + m[2,1]) / S
            w = (m[1,0] - m[0,1]) / S
    return Quaternion(x=x, y=y, z=z, w=w)

def quat_xyzw_to_rot(q: Quaternion) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return np.array([
        [1 - 2*(yy+zz),   2*(xy - wz),     2*(xz + wy)],
        [  2*(xy + wz), 1 - 2*(xx+zz),     2*(yz - wx)],
        [  2*(xz - wy),   2*(yz + wx),   1 - 2*(xx+yy)]
    ])

def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Quaternion multiply (xyzw)."""
    x1,y1,z1,w1 = q1; x2,y2,z2,w2 = q2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    return np.array([x,y,z,w])

def quat_conj(q: np.ndarray) -> np.ndarray:
    x,y,z,w = q
    return np.array([-x,-y,-z,w])

def rotate_vec_by_quat(v: np.ndarray, q: Quaternion) -> np.ndarray:
    """Rotate vector v by quaternion q (xyzw)."""
    qv = np.array([v[0], v[1], v[2], 0.0])
    qq = np.array([q.x, q.y, q.z, q.w])
    return quat_mul(quat_mul(qq, qv), quat_conj(qq))[:3]