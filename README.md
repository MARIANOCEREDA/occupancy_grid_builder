# ROS 2 Workspace - Occupancy Grid Builder

This workspace contains packages for building occupancy grids from sensor data including camera-LiDAR fusion, image processing, and SLAM.

## Prerequisites

- ROS 2 Jazzy
- Ubuntu 24.04 (or compatible)
- Python 3.12+
- colcon build tools

## Workspace Structure

```
src/
├── camera_lidar_fusion/      # Camera-LiDAR fusion library and ROS node
├── image_undistort/           # Image undistortion utilities
├── occupancy_grid_builder/    # Core occupancy grid builder
├── occupancy_grid_builder_bringup/  # Launch files and configuration
├── robot_description/         # Robot URDF and TF descriptions
├── segmentation_inference/    # Segmentation inference using Ultralytics
└── external/                  # External dependencies (KISS-ICP, etc.)
```

## Build Instructions

### 1. Install System Dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  python3-colcon-common-extensions \
  python3-rosdep \
  libeigen3-dev \
  libopencv-dev \
  libpcl-dev \
  nlohmann-json3-dev
```

### 2. Set Environment Variable for pip

```bash
export PIP_BREAK_SYSTEM_PACKAGES=1
```

Add this to your `~/.bashrc` to make it permanent:
```bash
echo 'export PIP_BREAK_SYSTEM_PACKAGES=1' >> ~/.bashrc
source ~/.bashrc
```

### 3. Register Custom rosdep.yaml

The workspace uses a custom rosdep file for the Ultralytics package:

```bash
echo "yaml file:///home/dev/ws/src/segmentation_inference/rosdep.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/70-rosdep-seg-inference.list
```

### 4. Install NumPy Constraint

```bash
pip install "numpy<2"
```

This ensures compatibility with packages that don't yet support NumPy 2.0.

### 5. Install ROS Dependencies

```bash
sudo apt update && rosdep update && rosdep install --from-paths src --ignore-src -y --as-root pip:false
```

This will install all required ROS 2 packages and the Ultralytics package via pip.

### 6. Build and source the Workspace

```bash
cd /home/dev/ws
colcon build && source install/setup.bash
```

## Running the System

### Launch the Occupancy Grid Builder

```bash
ros2 launch occupancy_grid_builder_bringup occupancy_grid_builder_bringup.launch.py
```

#### Launch Arguments

- `topic` (default: `/rslidar_points`): Input point cloud topic
- `visualize` (default: `true`): Enable RViz visualization
- `use_sim_time` (default: `false`): Use simulation time

Example with custom arguments:
```bash
ros2 launch occupancy_grid_builder_bringup occupancy_grid_builder_bringup.launch.py \
  topic:=/custom_points \
  visualize:=true \
  use_sim_time:=false
```

### Launching the local costmap (if needed)

```bash
ros2 run nav2_costmap_2d nav2_costmap_2d --ros-args -r __ns:=/local_costmap -r __node:=local_costmap --params-file /home/dev/ws/src/occupancy_grid_builder_bringup/config/local_costmap.yaml
```

Configure and activate the local costmap using lifecycle services:
```bash
ros2 lifecycle set local_costmap/local_costmap configure
ros2 lifecycle set local_costmap/local_costmap activate
```

## Cleaning Build Artifacts

```bash
cd /home/dev/ws
rm -rf build install log
```

### Using ROSBAGS

When using rosbags, make sure the --clock flag is enabled to synchronize time with the bag file:
```bash
ros2 bag play my_bag_file --clock
```

## Troubleshooting

### Issue: rosdep fails to find ultralytics
**Solution**: Make sure the custom rosdep.yaml is registered (step 3) and run `rosdep update`

### Issue: pip installation fails
**Solution**: Ensure `PIP_BREAK_SYSTEM_PACKAGES=1` is set (step 2)

## Package Details

### camera_lidar_fusion
- **C++ Library**: ROS-agnostic library for fusing camera masks with LiDAR point clouds
- **ROS Node**: Subscribes to segmentation masks and point clouds, publishes labeled point clouds

### segmentation_inference
- Segmentation inference using Ultralytics YOLO models
- Requires: `ultralytics` Python package (installed via rosdep)

### occupancy_grid_builder
- Core occupancy grid building from sensor data
- Includes SLAM integration (KISS-ICP)

### occupancy_grid_builder_bringup
- Launch files and configuration for the complete system
- RViz configuration included