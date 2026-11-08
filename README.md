# SLAM_and_navigation
SLAM and navigation for Unitree L2 LiDAR which will be used in KoNaR's project (Cerber)

## Point cloud from LiDAR

Compile:
```bash
colcon build --packages-select unitree_lidar_ros2
```

Run:
```bash
source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

## SLAM 3D (FAST-LIO)

Compile:
```bash
colcon build --packages-select fast_lio
```

Run:
```bash
source install/setup.bash
ros2 launch fast_lio mapping_l2.launch.py 
```

## SLAM 3D (DLIO)

Compile:
```bash
colcon build --packages-select direct_lidar_inertial_odometry
```

Run:
```bash
source install/setup.bash
ros2 launch direct_lidar_inertial_odometry dlio.launch.py
```

## SLAM 2D

Dependences:
```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
     ros-jazzy-slam-toolbox ros-jazzy-pointcloud-to-laserscan
```

Compile:
```bash
colcon build --packages-select mapping_2d
```

Run mapping 2D:
```bash
source install/setup.bash
ros2 launch mapping_2d slam2d.launch.py odometry:=dlio run_slam:=true
```
Can mapping with one of two SLAM 3D: dlio or fast_lio. Also can modify LiDAR localization on robot with additional parameters:
```bash
ros2 launch mapping_2d slam2d.launch.py odometry:=dlio run_slam:=true \
     lidar_x:=0.25 lidar_y:=0.0 lidar_z:=0.80 lidar_pitch_deg:=15 lidar_yaw_deg:=0
```

Save map:
```bash
ros2 run nav2_map_server map_saver_cli -f /home/jakub/Pulpit/SLAM_and_navigation/maps/mapa
```

Optionally, a pose graph, for further mapping or localization within slam_toolbox
```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
     "{filename: '/home/jakub/Pulpit/SLAM_and_navigation/maps/mapa_posegraph'}"
```

## Robot navigation based on map

Compile:
```bash
colcon build --packages-select mapping_2d
```

Run localization:
```bash
ros2 launch mapping_2d slam2d.launch.py odometry:=dlio run_slam:=false
```
Can mapping with one of two SLAM 3D: dlio or fast_lio. Also can modify LiDAR localization on robot with additional parameters:
```bash
ros2 launch mapping_2d slam2d.launch.py odometry:=dlio run_slam:=false \
     lidar_x:=0.25 lidar_y:=0.0 lidar_z:=0.80 lidar_pitch_deg:=15 lidar_yaw_deg:=0
```

Run navigation:
```bash
ros2 launch mapping_2d navigation.launch.py \
     map:=/home/jakub/Pulpit/SLAM_and_navigation/maps/mapa.yaml
```

## User manual for the DeepRobotics Lite3 quadruped robot

To connect to the robot via Ethernet use:
```bash
./connectRobot.sh
```

Run robot message translate package: 
```bash
ros2 launch transfer transfer_launch.py 
```

Command for the sit/stand movement:
```bash
ros2 topic pub --once /simple_cmd transfer_interfaces/msg/MotionSimpleCMD "{cmd_code: 1, size: 0, type: 1}"
```

Movement commands can be issued using:
```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

If the commands don't work (there is no transfer between the computer and the robot), check whether the recipient's IP address is correctly configured in the `jy_exe/conf/network.toml` file on the robot. It should be set to:
```bash
ip = 192.168.5.100
```