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
ros2 launch direct_lidar_inertial_odometry dlio.launch.py
```

## Mapowanie 2D

Zależności
```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
     ros-jazzy-slam-toolbox ros-jazzy-pointcloud-to-laserscan
```

Odpalanie mapowania 2D
```bash
ros2 launch mapping_2d slam2d.launch.py odometry:=dlio run_slam:=true

ros2 launch mapping_2d slam2d.launch.py odometry:=dlio run_slam:=true \
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

## Nawigacja robota na podstawie mapy

```bash
ros2 launch mapping_2d slam2d.launch.py odometry:=dlio run_slam:=false
ros2 launch mapping_2d navigation.launch.py map:=/home/jakub/Pulpit/SLAM_and_navigation/maps/mapa.yaml

```

## Instrukcja użycia do robota kroczącego Lite3

Do połączenia się po Ethernet z robotem należy użyć
```bash
./connectRobot.sh
```

Następnie odpalamy paczkę
```bash
ros2 launch transfer transfer_launch.py 
```

Komendy do ruchu siad/wstań
```bash
ros2 topic pub --once /simple_cmd transfer_interfaces/msg/MotionSimpleCMD "{cmd_code: 1, size: 0, type: 1}"
```

Komendy ruchu można podawać za pomocą 
```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Jeśli komendy nie działają (nie ma transfery między komputerem a robotem), należy sprawdzić czy na robocie w pliku jy_exe/conf/network.toml ip odbiorcy jest skonfigurowane poprawnie
```bash
ip = 192.168.5.100
```