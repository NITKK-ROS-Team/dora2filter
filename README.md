# dora2filter
A filter using the dot product of direction vectors from two adjacent points and their midpoint vector.

## Installation

## Dependencies

- ROS2 Humble or later
- laser_filters package
- socat
- (Optional) [YDLIDAR-SDK](https://github.com/YDLIDAR/YDLidar-SDK)

```bash
mkdir ~/dora2filter_ws/src -p
cd ~/dora2filter_ws/src

git clone https://github.com/NITKK-ROS-Team/dora2filter.git
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git -b humble
# git clone https://github.com/Ar-Ray-code/tmini_plus_emulator.git # Optional : use filter demo without LiDAR

cd ~/dora2filter_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Usage

### Terminal1

Start the LiDAR driver

```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py params_file:=<path to YDLIDAR config>
```

### Terminal2

Start the filter node

```bash
ros2 run laser_filters scan_to_scan_filter_chain --ros-args -r __node:=dora2_filter_node --params-file ./src/dora2filter/config/dora2_filter.yaml
```

<img width="1731" height="554" alt="image" src="https://github.com/user-attachments/assets/a2e8d7ab-355f-4f3b-b176-35197e42b09a" />


## References

- [NITIC-Robot-Club/natto_library](https://github.com/NITIC-Robot-Club/natto_library/blob/main/src/sensing/natto_laser_filter/src/laser_filter.cpp)

Special thanks : [@Doraemonjayo](https://github.com/Doraemonjayo)