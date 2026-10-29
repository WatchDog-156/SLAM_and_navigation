# SLAM_and_navigation
SLAM and navigation for Unitree L2 LiDAR which will be used in KoNaR's project (Cerber)


## Odpalanie LiDAR'u

```bash
source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

## Odpalanie mapowania 3D (FAST-LIO)

```bash
source install/setup.bash
ros2 launch fast_lio mapping_l2.launch.py 
```

## Odpalanie mapowania 3D (DLIO)

```bash
source install/setup.bash
ros2 launch direct_lidar_inertial_odometry dlio.launch.py \
     rviz:=true pointcloud_topic:=/unilidar/cloud imu_topic:=/unilidar/imu
```

## Mapowanie 2D

Zależności
```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
     ros-jazzy-slam-toolbox ros-jazzy-pointcloud-to-laserscan
```

Odpalanie mapowania 2D
```bash
ros2 launch ~/l2_slam2d/slam2d.launch.py \
     lidar_x:=0.25 lidar_y:=0.0 lidar_z:=0.80 lidar_pitch_deg:=15 lidar_yaw_deg:=0
```

Uruchamiaj to razem z lidarem i mapping_l2.launch.py. W RViz ustaw Fixed Frame na map i dodaj Map (/map) oraz LaserScan (/scan).

Zapis mapy
```bash
ros2 run nav2_map_server map_saver_cli -f /home/jakub/Pulpit/SLAM_and_navigation/maps/mapa
```

Opcjonalnie graf pozycji, do dalszego mapowania lub lokalizacji w slam_toolbox
```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/jakub/Pulpit/SLAM_and_navigation/maps/mapa_posegraph'}"
```