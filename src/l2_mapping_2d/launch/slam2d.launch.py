"""2D SLAM: FAST-LIO (odometria) + pointcloud_to_laserscan + slam_toolbox.

Uruchamiaj RAZEM z: ros2 launch unitree_lidar_ros2 launch.py  oraz  ros2 launch fast_lio mapping_l2.launch.py
Przyklad:
  ros2 launch ~/l2_slam2d/slam2d.launch.py lidar_x:=0.25 lidar_z:=0.80 lidar_pitch_deg:=15
Konwencja: pitch > 0 = lidar patrzy w DOL (os x lidaru pochylona w dol).
base_link = srodek robota, na poziomie podlogi (z=0), x do przodu.
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

pkg = get_package_share_directory('l2_mapping_2d')

def quat_zy(yaw, pitch):
    """Kwaternion (x,y,z,w) dla R = Rz(yaw) * Ry(pitch)."""
    sz, cz = math.sin(yaw / 2), math.cos(yaw / 2)
    sy, cy = math.sin(pitch / 2), math.cos(pitch / 2)
    return (-sz * sy, cz * sy, sz * cy, cz * cy)


def rot_zy(yaw, pitch):
    cz, sz, cy, sy = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    # Rz * Ry
    return [[cz * cy, -sz, cz * sy],
            [sz * cy, cz, sz * sy],
            [-sy, 0.0, cy]]


def setup(context):
    f = lambda n: float(LaunchConfiguration(n).perform(context))
    x, y, z = f('lidar_x'), f('lidar_y'), f('lidar_z')
    pitch, yaw = math.radians(f('lidar_pitch_deg')), math.radians(f('lidar_yaw_deg'))

    # T = base_link -> lidar (== body FAST-LIO, roznica ~1 cm pomijalna)
    qx, qy, qz, qw = quat_zy(yaw, pitch)
    R = rot_zy(yaw, pitch)
    t = (x, y, z)
    # odwrotnosc: lidar -> base_link
    inv_t = tuple(-sum(R[j][i] * t[j] for j in range(3)) for i in range(3))
    inv_q = (-qx, -qy, -qz, qw)

    def static_tf(tr, q, parent, child):
        return Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['--x', str(tr[0]), '--y', str(tr[1]), '--z', str(tr[2]),
                       '--qx', str(q[0]), '--qy', str(q[1]), '--qz', str(q[2]), '--qw', str(q[3]),
                       '--frame-id', parent, '--child-frame-id', child])

    nodes = [
        # base_link -> unilidar_lidar (montaz)
        static_tf(t, (qx, qy, qz, qw), 'base_link', 'unilidar_lidar'),
        # odom -> camera_init: wypoziomowanie (FAST-LIO startuje w ukladzie pochylonego lidaru)
        static_tf(t, (qx, qy, qz, qw), 'odom', 'camera_init'),
        # body -> base_link: odwrotnosc montazu
        static_tf(inv_t, inv_q, 'body', 'base_link'),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[('cloud_in', '/unilidar/cloud'), ('scan', '/scan')],
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.2,
                'min_height': f('scan_min_height'),   # m nad base_link (podloga = 0)
                'max_height': f('scan_max_height'),
                'angle_min': -math.pi, 'angle_max': math.pi,
                'angle_increment': 0.0087,
                'scan_time': 0.1,
                'range_min': 0.25, 'range_max': 15.0,
                'use_inf': True, 'inf_epsilon': 1.0,
            }]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
            launch_arguments={'slam_params_file': os.path.join(pkg, 'config', 'slam_toolbox.yaml'),
                              'use_sim_time': 'false'}.items()),
    ]
    return nodes


def generate_launch_description():
    args = [
        DeclareLaunchArgument('lidar_x', default_value='0.20', description='m do przodu od base_link'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.60', description='wysokosc lidaru nad podloga [m]'),
        DeclareLaunchArgument('lidar_pitch_deg', default_value='15.0', description='pochylenie w dol [deg]'),
        DeclareLaunchArgument('lidar_yaw_deg', default_value='0.0'),
        DeclareLaunchArgument('scan_min_height', default_value='0.15'),
        DeclareLaunchArgument('scan_max_height', default_value='0.60'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=setup)])