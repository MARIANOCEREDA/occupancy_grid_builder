# Occupancy Grid Builder - ROS2 Lifecycle Node

This package provides a ROS2 lifecycle node wrapper for the occupancy grid builder C++ library. The node converts 3D point clouds into 2D occupancy grids for robot navigation and mapping.

## Features

- **Lifecycle Node**: Full lifecycle management (configure, activate, deactivate, cleanup, shutdown)
- **Composable**: Can be loaded as a component in a container for efficient inter-process communication
- **Standalone**: Also available as a standalone executable
- **TF2 Integration**: Automatic coordinate frame transformations
- **PCL Support**: Processes colored point clouds (XYZRGB)

## Building

```bash
cd /path/to/workspace
colcon build --packages-select occupancy_grid_builder
source install/setup.bash
```

## Running

### Option 1: Using Launch File (Recommended)

The launch file automatically manages lifecycle transitions:

```bash
ros2 launch occupancy_grid_builder occupancy_grid_builder.launch.py
```

With custom parameters:

```bash
ros2 launch occupancy_grid_builder occupancy_grid_builder.launch.py \
  params_file:=/path/to/your/params.yaml \
  use_sim_time:=true
```

### Option 2: Standalone Executable

Run the node directly:

```bash
ros2 run occupancy_grid_builder occupancy_grid_builder_standalone
```

Then manually manage lifecycle transitions:

```bash
# Configure the node
ros2 lifecycle set /occupancy_grid_builder configure

# Activate the node
ros2 lifecycle set /occupancy_grid_builder activate

# Deactivate when needed
ros2 lifecycle set /occupancy_grid_builder deactivate
```

### Option 3: Composable Node

Load into a component container:

```bash
# Start a component container
ros2 run rclcpp_components component_container

# In another terminal, load the component
ros2 component load /ComponentManager occupancy_grid_builder occupancy_grid_builder::OccupancyGridBuilderNode
```

## Configuration

Edit `config/occupancy_grid_builder.yaml` to configure the node:

```yaml
occupancy_grid_builder:
  ros__parameters:
    # Topics
    pointcloud_topic: "/pointcloud"
    grid_topic: "/occupancy_grid"
    
    # Frames
    map_frame: "map"
    sensor_frame: "camera_link"
    
    # Grid parameters
    resolution: 0.1      # meters per cell
    width: 1000         # cells
    height: 1000        # cells
    origin_x: -50.0     # meters
    origin_y: -50.0     # meters
```

## Topics

### Subscribed Topics

- `pointcloud_topic` (sensor_msgs/PointCloud2): Input colored point cloud

### Published Topics

- `grid_topic` (nav_msgs/OccupancyGrid): Output 2D occupancy grid
  - Values: 0 (free), 100 (occupied), -1 (unknown)

## Lifecycle States

The node follows the standard ROS2 lifecycle:

1. **Unconfigured** → Configure → **Inactive**
2. **Inactive** → Activate → **Active** (processing data)
3. **Active** → Deactivate → **Inactive**
4. **Inactive** → Cleanup → **Unconfigured**

## Monitoring

Check node status:

```bash
# List lifecycle nodes
ros2 lifecycle list

# Get current state
ros2 lifecycle get /occupancy_grid_builder

# Monitor all state changes
ros2 topic echo /occupancy_grid_builder/transition_event
```

## Troubleshooting

### Node Not Processing Data

Make sure the node is in the **Active** state:

```bash
ros2 lifecycle get /occupancy_grid_builder
```

If not active, configure and activate it:

```bash
ros2 lifecycle set /occupancy_grid_builder configure
ros2 lifecycle set /occupancy_grid_builder activate
```

### TF Transform Errors

Ensure your TF tree is properly set up and the transforms between `sensor_frame` and `map_frame` are being published.

Check available transforms:

```bash
ros2 run tf2_tools view_frames
```

### No Point Cloud Data

Verify the point cloud topic is publishing:

```bash
ros2 topic echo /pointcloud --once
ros2 topic hz /pointcloud
```
