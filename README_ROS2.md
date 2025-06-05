# KISS-SLAM ROS2 Package

This is the ROS2 version of KISS-SLAM: A Simple, Robust, and Accurate 3D LiDAR SLAM System With Enhanced Generalization Capabilities.

## Installation

1. Make sure you have ROS2 installed (tested with ROS2 Humble/Iron)
2. Clone this repository into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url> kiss_slam
   ```

3. Install dependencies:
   ```bash
   cd kiss_slam
   
   # Install ROS2 dependencies via rosdep
   rosdep install --from-paths . --ignore-src -r -y
   
   # Install KISS-SLAM specific Python dependencies via pip
   pip3 install "kiss-icp>=1.2.3" "map_closures>=2.0.2" "open3d>=0.19.0" "pydantic>=2" "pydantic-settings"
   
   # OR use the provided script
   ./install_dependencies.sh
   ```

4. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select kiss_slam
   source install/setup.bash
   ```

## Dependencies

### ROS2 Dependencies (installed via rosdep)
- rclpy, sensor_msgs, geometry_msgs, nav_msgs, tf2_ros, visualization_msgs
- python3-numpy, python3-scipy, python3-yaml, python3-tqdm, python3-pydantic

### KISS-SLAM Specific Dependencies (installed via pip)
- `kiss-icp>=1.2.3`: Core ICP odometry algorithm
- `map_closures>=2.0.2`: Loop closure detection
- `open3d>=0.19.0`: 3D geometry processing
- `pydantic>=2`: Configuration validation
- `pydantic-settings`: Settings management

**Note**: These dependencies are not available through rosdep and must be installed via pip3.

## Usage

### Running the SLAM Node

Launch the KISS-SLAM node with default parameters:
```bash
ros2 launch kiss_slam kiss_slam.launch.py
```

Launch with custom parameters:
```bash
ros2 launch kiss_slam kiss_slam.launch.py input_topic:=/velodyne_points output_frame:=map
```

### Parameters

- `input_topic` (default: `/points`): Input PointCloud2 topic
- `output_frame` (default: `map`): Output frame for SLAM results  
- `base_frame` (default: `base_link`): Base frame of the robot
- `lidar_frame` (default: `lidar`): LiDAR sensor frame
- `config_file` (default: `''`): Path to KISS-SLAM configuration file
- `visualize` (default: `true`): Enable visualization outputs
- `publish_rate` (default: `10.0`): Rate for publishing SLAM results (Hz)
- `use_sim_time` (default: `false`): Use simulation time

### Published Topics

- `/kiss_slam/odometry` (`nav_msgs/Odometry`): SLAM odometry
- `/kiss_slam/path` (`nav_msgs/Path`): Robot trajectory path
- `/kiss_slam/pose` (`geometry_msgs/PoseStamped`): Current robot pose
- `/kiss_slam/local_map` (`sensor_msgs/PointCloud2`): Current local map (if visualization enabled)
- `/kiss_slam/loop_closures` (`visualization_msgs/MarkerArray`): Loop closure visualization (if visualization enabled)

### TF Frames

The node publishes TF transforms from `output_frame` to `base_frame`.

## Example Usage

### With a ROS2 bag
```bash
# Terminal 1: Play bag file
ros2 bag play your_lidar_bag.mcap

# Terminal 2: Run KISS-SLAM
ros2 launch kiss_slam kiss_slam.launch.py input_topic:=/your_lidar_topic
```

### With live sensor data
```bash
# Make sure your LiDAR driver is publishing PointCloud2 messages
ros2 launch kiss_slam kiss_slam.launch.py input_topic:=/velodyne_points
```

### Visualization in RViz2
```bash
# Open RViz2 and add the following displays:
# - TF
# - Path (/kiss_slam/path)
# - PointCloud2 (/kiss_slam/local_map)
# - MarkerArray (/kiss_slam/loop_closures)
rviz2
```

## Configuration

You can create custom configuration files for KISS-SLAM and pass them via the `config_file` parameter. See the original KISS-SLAM documentation for configuration options.

## Troubleshooting

### ModuleNotFoundError: No module named 'kiss_icp'
Make sure you've installed the pip dependencies:
```bash
pip3 install "kiss-icp>=1.2.3" "map_closures>=2.0.2" "open3d>=0.19.0" "pydantic>=2" "pydantic-settings"
```

### Package build fails
Make sure all rosdep dependencies are installed:
```bash
rosdep install --from-paths . --ignore-src -r -y
``` 