# """2D SLAM: FAST-LIO (odometria) + pointcloud_to_laserscan + slam_toolbox.

# Uruchamiaj RAZEM z: ros2 launch unitree_lidar_ros2 launch.py  oraz  ros2 launch fast_lio mapping_l2.launch.py
# Przyklad:
#   ros2 launch ~/l2_slam2d/slam2d.launch.py lidar_x:=0.25 lidar_z:=0.80 lidar_pitch_deg:=15
# Konwencja: pitch > 0 = lidar patrzy w DOL (os x lidaru pochylona w dol).
# base_link = srodek robota, na poziomie podlogi (z=0), x do przodu.
# """
# import math
# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node

# pkg = get_package_share_directory('l2_mapping_2d')

# def quat_zy(yaw, pitch):
#     """Kwaternion (x,y,z,w) dla R = Rz(yaw) * Ry(pitch)."""
#     sz, cz = math.sin(yaw / 2), math.cos(yaw / 2)
#     sy, cy = math.sin(pitch / 2), math.cos(pitch / 2)
#     return (-sz * sy, cz * sy, sz * cy, cz * cy)


# def rot_zy(yaw, pitch):
#     cz, sz, cy, sy = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
#     # Rz * Ry
#     return [[cz * cy, -sz, cz * sy],
#             [sz * cy, cz, sz * sy],
#             [-sy, 0.0, cy]]


# def setup(context):
#     f = lambda n: float(LaunchConfiguration(n).perform(context))
#     x, y, z = f('lidar_x'), f('lidar_y'), f('lidar_z')
#     pitch, yaw = math.radians(f('lidar_pitch_deg')), math.radians(f('lidar_yaw_deg'))

#     # T = base_link -> lidar (== body FAST-LIO, roznica ~1 cm pomijalna)
#     qx, qy, qz, qw = quat_zy(yaw, pitch)
#     R = rot_zy(yaw, pitch)
#     t = (x, y, z)
#     # odwrotnosc: lidar -> base_link
#     inv_t = tuple(-sum(R[j][i] * t[j] for j in range(3)) for i in range(3))
#     inv_q = (-qx, -qy, -qz, qw)

#     def static_tf(tr, q, parent, child):
#         return Node(
#             package='tf2_ros', executable='static_transform_publisher',
#             arguments=['--x', str(tr[0]), '--y', str(tr[1]), '--z', str(tr[2]),
#                        '--qx', str(q[0]), '--qy', str(q[1]), '--qz', str(q[2]), '--qw', str(q[3]),
#                        '--frame-id', parent, '--child-frame-id', child])

#     nodes = [
#         # base_link -> unilidar_lidar (montaz)
#         static_tf(t, (qx, qy, qz, qw), 'base_link', 'unilidar_lidar'),
#         # odom -> camera_init: wypoziomowanie (FAST-LIO startuje w ukladzie pochylonego lidaru)
#         static_tf(t, (qx, qy, qz, qw), 'odom', 'camera_init'),
#         # body -> base_link: odwrotnosc montazu
#         static_tf(inv_t, inv_q, 'body', 'base_link'),
#         Node(
#             package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
#             name='pointcloud_to_laserscan',
#             remappings=[('cloud_in', '/unilidar/cloud'), ('scan', '/scan')],
#             parameters=[{
#                 'target_frame': 'base_link',
#                 'transform_tolerance': 0.2,
#                 'min_height': f('scan_min_height'),   # m nad base_link (podloga = 0)
#                 'max_height': f('scan_max_height'),
#                 'angle_min': -math.pi, 'angle_max': math.pi,
#                 'angle_increment': 0.0087,
#                 'scan_time': 0.1,
#                 'range_min': 0.25, 'range_max': 15.0,
#                 'use_inf': True, 'inf_epsilon': 1.0,
#             }]),
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(os.path.join(
#                 get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
#             launch_arguments={'slam_params_file': os.path.join(pkg, 'config', 'slam_toolbox.yaml'),
#                               'use_sim_time': 'false'}.items()),
#     ]
#     return nodes


# def generate_launch_description():
#     args = [
#         DeclareLaunchArgument('lidar_x', default_value='0.20', description='m do przodu od base_link'),
#         DeclareLaunchArgument('lidar_y', default_value='0.0'),
#         DeclareLaunchArgument('lidar_z', default_value='0.60', description='wysokosc lidaru nad podloga [m]'),
#         DeclareLaunchArgument('lidar_pitch_deg', default_value='15.0', description='pochylenie w dol [deg]'),
#         DeclareLaunchArgument('lidar_yaw_deg', default_value='0.0'),
#         DeclareLaunchArgument('scan_min_height', default_value='0.15'),
#         DeclareLaunchArgument('scan_max_height', default_value='0.60'),
#     ]
#     return LaunchDescription(args + [OpaqueFunction(function=setup)])

"""Wspolny frontend 2D dla FAST-LIO albo DLIO + pointcloud_to_laserscan (+ opcjonalnie slam_toolbox).

Mapowanie (slam_toolbox):
  ros2 launch mapping_2d slam2d.launch.py odometry:=fast_lio  run_slam:=true
  ros2 launch mapping_2d slam2d.launch.py odometry:=dlio      run_slam:=true
Nawigacja (AMCL w Nav2, slam_toolbox WYLACZONY):
  ros2 launch mapping_2d slam2d.launch.py odometry:=dlio      run_slam:=false

Lidar uruchamiasz osobno: ros2 launch unitree_lidar_ros2 launch.py
NIE uruchamiaj osobno fast_lio ani dlio - ten launch startuje wybrany algorytm sam.

Oba algorytmy publikuja odometrie na /odom (remap), wiec Nav2 zostaje z `odom_topic: odom`.
Konwencja: pitch > 0 = lidar patrzy w DOL. base_link = srodek robota na podlodze, x do przodu.
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

IMU_IN_LIDAR = (0.007698, 0.014655, -0.00667)


def quat_zy(yaw, pitch):
    """Kwaternion (x,y,z,w) dla R = Rz(yaw) * Ry(pitch)."""
    sz, cz = math.sin(yaw / 2), math.cos(yaw / 2)
    sy, cy = math.sin(pitch / 2), math.cos(pitch / 2)
    return (-sz * sy, cz * sy, sz * cy, cz * cy)


def rot_zy(yaw, pitch):
    cz, sz, cy, sy = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    return [[cz * cy, -sz, cz * sy],
            [sz * cy, cz, sz * sy],
            [-sy, 0.0, cy]]


def matvec(R, v):
    return [sum(R[i][j] * v[j] for j in range(3)) for i in range(3)]


def static_tf(tr, q, parent, child):
    return Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', str(tr[0]), '--y', str(tr[1]), '--z', str(tr[2]),
                   '--qx', str(q[0]), '--qy', str(q[1]), '--qz', str(q[2]), '--qw', str(q[3]),
                   '--frame-id', parent, '--child-frame-id', child])


def setup(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)
    f = lambda n: float(arg(n))

    odometry = arg('odometry').lower()
    if odometry not in ('fast_lio', 'dlio'):
        raise RuntimeError("odometry musi byc 'fast_lio' albo 'dlio', jest: " + odometry)

    pkg = get_package_share_directory('mapping_2d')
    cloud_topic, imu_topic = arg('cloud_topic'), arg('imu_topic')

    t = [f('lidar_x'), f('lidar_y'), f('lidar_z')]
    pitch, yaw = math.radians(f('lidar_pitch_deg')), math.radians(f('lidar_yaw_deg'))
    R = rot_zy(yaw, pitch)
    qx, qy, qz, qw = quat_zy(yaw, pitch)

    nodes = []

    if odometry == 'fast_lio':
        # FAST-LIO publikuje tylko camera_init -> body, wiec lacznik odom/base_link robimy statycznie.
        inv_t = [-sum(R[j][i] * t[j] for j in range(3)) for i in range(3)]
        inv_q = (-qx, -qy, -qz, qw)
        nodes += [
            static_tf(t, (qx, qy, qz, qw), 'base_link', 'unilidar_lidar'),
            static_tf(t, (qx, qy, qz, qw), 'odom', 'camera_init'),
            static_tf(inv_t, inv_q, 'body', 'base_link'),
            Node(
                package='fast_lio', executable='fastlio_mapping', name='fastlio_mapping',
                output='screen',
                parameters=[os.path.join(get_package_share_directory('fast_lio'),
                                         'config', 'unitree_l2.yaml')],
                remappings=[('/Odometry', '/odom')]),
        ]
    else:
        # DLIO samo publikuje odom -> base_link oraz base_link -> unilidar_lidar / unilidar_imu
        # na podstawie extrinsics, wiec ZADNYCH statycznych TF tu nie dodajemy (byloby podwojne TF).
        # base_link = podloga pod robotem, wiec extrinsics = montaz lidaru wzgledem podlogi.
        dlio = get_package_share_directory('direct_lidar_inertial_odometry')
        imu_t = [t[i] + v for i, v in enumerate(matvec(R, IMU_IN_LIDAR))]
        flatR = [R[i][j] for i in range(3) for j in range(3)]
        nodes.append(Node(
            package='direct_lidar_inertial_odometry', executable='dlio_odom_node',
            name='dlio_odom_node', output='screen',
            parameters=[
                os.path.join(dlio, 'config', 'dlio.yaml'),
                os.path.join(dlio, 'config', 'params.yaml'),
                {
                    'extrinsics/baselink2lidar/t': t,
                    'extrinsics/baselink2lidar/R': flatR,
                    'extrinsics/baselink2imu/t': imu_t,
                    'extrinsics/baselink2imu/R': flatR,
                    # wyrownanie do grawitacji przy starcie (w params.yaml bylo false)
                    'odom/imu/approximateGravity': True,
                }],
            remappings=[
                ('pointcloud', cloud_topic),
                ('imu', imu_topic),
                ('odom', '/odom'),
                ('pose', 'dlio/odom_node/pose'),
                ('path', 'dlio/odom_node/path'),
                ('kf_pose', 'dlio/odom_node/keyframes'),
                ('kf_cloud', 'dlio/odom_node/pointcloud/keyframe'),
                ('deskewed', 'dlio/odom_node/pointcloud/deskewed'),
            ]))

    nodes.append(Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[('cloud_in', cloud_topic), ('scan', '/scan')],
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
        }]))

    # slam_toolbox tylko przy mapowaniu. Przy nawigacji (AMCL) MUSI byc wylaczony,
    # bo oba wezly publikuja map->odom.
    if arg('run_slam').lower() == 'true':
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
            launch_arguments={'slam_params_file': os.path.join(pkg, 'config', 'slam_toolbox.yaml'),
                              'use_sim_time': 'false'}.items()))
    return nodes


def generate_launch_description():
    current_pkg = FindPackageShare('mapping_2d')

    args = [
        DeclareLaunchArgument('odometry', default_value='fast_lio',
                              description="'fast_lio' albo 'dlio'"),
        DeclareLaunchArgument('run_slam', default_value='true',
                              description='true = mapowanie (slam_toolbox), false = do nawigacji'),
        DeclareLaunchArgument('cloud_topic', default_value='/unilidar/cloud'),
        DeclareLaunchArgument('imu_topic', default_value='/unilidar/imu'),
        DeclareLaunchArgument('lidar_x', default_value='0.0', description='m do przodu od base_link'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.20', description='wysokosc lidaru nad podloga [m]'),
        DeclareLaunchArgument('lidar_pitch_deg', default_value='0.0', description='pochylenie w dol [deg]'),
        DeclareLaunchArgument('lidar_yaw_deg', default_value='0.0'),
        DeclareLaunchArgument('scan_min_height', default_value='0.15'),
        DeclareLaunchArgument('scan_max_height', default_value='0.50'),
    ]

    rviz_config_path = PathJoinSubstitution([current_pkg, 'rviz', 'mapping2d.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='dlio_rviz',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription( args + [
        OpaqueFunction(function=setup),
        rviz_node,
    ])