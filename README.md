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
git clone https://github.com/Ar-Ray-code/tmini_plus_emulator.git
git clone https://github.com/NITKK-ROS-Team/dora2filter.git
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git -b humble

cd ~/dora2filter_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Usage

### Terminal1

Create virtual serial ports

```bash
socat -d -d pty,raw,echo=0,link=/tmp/ttyEMUL pty,raw,echo=0,link=/tmp/ttyROS
```

### Terminal2

```bash
./build/tmini_plus_emulator_cpp/tmini_plus_emulator /tmp/ttyEMUL src/tmini_plus_emulator/data/example_data.txt
```

### Terminal3

Start the LiDAR driver

```bash
ros2 launch ydlidar_ros2_driver  ydlidar_launch.py  params_file:=./src/tmini_plus_emulator/data/lidar_params.yaml
```

### Terminal4

Start the filter node

```bash
ros2 run laser_filters scan_to_scan_filter_chain --ros-args -r __node:=dora2_filter_node --params-file ./src/dora2filter/config/dora2_filter.yaml
```
