## Coordinate System Definition

The coordinate system of this LiDAR is defined in a right-handed coordinate system. Let the LiDAR point cloud coordinate system be L, and the IMU coordinate system be I.

The origin of the LiDAR point cloud coordinate system is located at the center of the bottom mounting surface of the LiDAR. Its +X-axis is opposite to the direction of the bottom cable outlet, its +Y-axis is obtained by rotating the +X-axis counterclockwise by 90 degrees, and its +Z-axis is perpendicular to the bottom surface.

The three coordinate axes of the IMU coordinate system are parallel to the corresponding coordinate axes of the point cloud coordinate system, and the two only have a translation of the origin position. The origin of the IMU coordinate system in the LiDAR point cloud coordinate system is (in meters): [-0.007698, -0.014655, 0.00667].

According to the standard transformation matrix method, the pose transformation from the LiDAR point cloud coordinate system L to the IMU coordinate system I is:

$$
T_{LI} =
\begin{bmatrix}
1 & 0 & 0 & -0.007698 \\
0 & 1 & 0 & -0.014655 \\
0 & 0 & 1 & 0.00667 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$


## C++ SDK

### Compilation

You can compile the sample programs of this project according to the standard compilation method of cmake projects:
```bash
cd unitree_lidar_sdk

mkdir build

cd build

cmake .. && make -j2
```

### Configuring Work Mode
The LiDAR can be configured in various working modes by default, including standard FOV or wide-angle FOV, 3D mode or 2D mode, IMU enabled or disabled, Ethernet or serial connection, etc.

We can configure the working mode through the host computer, or we can also configure the working mode through the following interface in `unitree_lidar_sdk.h`:
```
virtual void setLidarWorkMode(uint32_t mode) = 0;
```

Setting the working mode is implemented through a `uint32_t` integer variable, with each bit corresponding to the switching of a function. According to the bit position from low to high, the corresponding functions for positions 0 or 1 are shown in the following table:

|Bit Position|Function|Value 0|Value 1|
|---|---|---|----|
|0|Switch between standard FOV and wide-angle FOV|Standard FOV (180°)|Wide-angle FOV (192°)|
|1|Switch between 3D and 2D measurement modes|3D measurement mode|2D measurement mode|
|2|Enable or disable IMU|Enable IMU|Disable IMU|
|3|Switch between Ethernet mode and serial mode|Ethernet mode|Serial mode|
|4|Switch between lidar power-on default start mode|Power on and start automatically|Power on and wait for start command without rotation|
|5-31|Reserved|Reserved|Reserved|

The common usage mode is standard FOV + 3D measurement + enable IMU + power on self-start, which means these bit positions are all kept at 0. We only need to determine whether to use Ethernet connection mode or serial connection mode. For example, in the following sample program,
- If using Ethernet connection mode, configure the work mode to 0 (i.e., all bit positions of the integer variable are equal to 0)
- If using serial connection mode, configure the work mode to 8 (i.e., the third bit position of the integer variable is equal to 1, and the other 0-31 bit positions are all equal to 0)

The default factory lidar work mode is 0, i.e., Ethernet communication mode.

### Running with Ethernet

```bash
lidar_ip = 192.168.1.62
local_ip = 192.168.1.2
```

### Running with Serial Port

```
"/dev/ttyACM1"
```


## How to Use the ROS2 Package

### Configuration

The default communication method for the LiDAR is Ethernet mode. If you need to modify the working mode, you need to change the corresponding parameters in the configuration file. The path to the configuration file is:
```
unitree_lidar_ros2/src/unitree_lidar_ros2/launch/launch.py
```

If you have special needs, such as changing the cloud topic name or IMU topic name, you can also configure them in the configuration file.

The default cloud topic and its coordinate system name are:
- Topic name: "unilidar/cloud"
- Coordinate system: "unilidar_lidar"

The default IMU topic and its coordinate system name are:
- Topic name: "unilidar/imu"
- Coordinate system: "unilidar_imu"

### Compilation and running

Compile:

```bash
colcon build --packages-select unitree_lidar_ros2
```

### Running

Run:

```bash
source install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

In the Rviz window, you will see our LiDAR point cloud.