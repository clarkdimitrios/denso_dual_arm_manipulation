import subprocess
import xml.etree.ElementTree as ET

from math import radians, degrees
import transforms3d
from geometry_msgs.msg import Pose

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
    return transforms3d.euler.quat2euler(q, axes='sxyz')

def quat_to_euler_deg(q):
    return degrees(transforms3d.euler.quat2euler(q, axes='sxyz'))