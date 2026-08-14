from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import Transition
import os

import xacro


def generate_launch_description():

    # --- Arguments ---
    topic_arg = DeclareLaunchArgument(
        'topic',
        default_value='/rslidar_points',
        description='Input point cloud topic'
    )
    visualize_arg = DeclareLaunchArgument(
        'visualize',
        default_value='true',
        description='Visualize in RViz'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (bag) clock'
    )
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Robot base frame'
    )
    lidar_odom_frame_arg = DeclareLaunchArgument(
        'lidar_odom_frame',
        default_value='odom',
        description='Odometry output frame'
    )

    # --- Robot description ---
    robot_description_pkg_name = 'robot_description'
    pkg_share = get_package_share_directory(robot_description_pkg_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot_description.urdf.xacro')

    robot_description = xacro.process_file(urdf_file).toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # --- KISS-ICP node ---
    kiss_icp_component = ComposableNode(
        package='kiss_icp',
        plugin='kiss_icp_ros::OdometryServer',
        name='kiss_icp_node',
        remappings=[
            ('pointcloud_topic', LaunchConfiguration('topic')),
        ],
        parameters=[{
            'base_frame': LaunchConfiguration('base_frame'),
            'lidar_odom_frame': LaunchConfiguration('lidar_odom_frame'),
            'publish_odom_tf': True,
            'invert_odom_tf': False,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'visualize': False
        }],
    )

    # --- RViz node ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory('occupancy_grid_builder_bringup'), 'rviz', 'kiss_icp.rviz')],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(LaunchConfiguration('visualize'))
    )

    # --- Image undistort Composable Node ---
    image_undistort_component = ComposableNode(
        package='image_undistort',
        plugin='image_undistort::ImageUndistort',
        name='image_undistort_node',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'resize_output': False
        }],
    )

    # --- Segmentation Inference ONNX Composable Node ---
    segmentation_inference_onnx_component = ComposableNode(
        package='segmentation_inference_onnx_ros',
        plugin='yolo_onnx_inference_ros::InferenceNode',
        name='segmentation_inference_onnx_node',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'model_path': os.path.join(
                get_package_share_directory('segmentation_inference_onnx_ros'), 
                'models', 
                'yolo11n-seg.onnx'
            ),
            'device_type': 'CPU',
        }],
    )

    # --- Camera-LiDAR Fusion Composable Node ---
    camera_lidar_fusion_component = ComposableNode(
        package='camera_lidar_fusion',
        plugin='camera_lidar_fusion::CameraLidarFusionNode',
        name='camera_lidar_fusion_node',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # --- Occupancy Grid Builder Composable Node ---
    occupancy_grid_builder_component = ComposableNode(
        package='occupancy_grid_builder',
        plugin='occupancy_grid_builder::OccupancyGridBuilderNode',
        name='occupancy_grid_builder_node',
        parameters=[
            os.path.join(
                get_package_share_directory('occupancy_grid_builder_bringup'), 
                'config', 
                'occupancy_grid_builder.yaml'
            ),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    # --- EKF node ---
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('occupancy_grid_builder_bringup'), 'config', 'ekf.yaml'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # ----- Map to odom transform broadcaster node -----
    map_to_odom_publish_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_broadcaster',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # ----- Nav2 Local Costmap Node -----
    local_costmap_params_file = os.path.join(
        get_package_share_directory('occupancy_grid_builder_bringup'), 
        'config', 
        'local_costmap.yaml'
    )
    local_costmap_node = LifecycleNode(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        namespace='local_costmap',
        name='local_costmap',
        output='screen',
        parameters=[local_costmap_params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    perception_container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            kiss_icp_component,
            image_undistort_component,
            segmentation_inference_onnx_component,
            camera_lidar_fusion_component,
            occupancy_grid_builder_component
        ],
        output='screen',
    )

    # --- Lifecycle manager for all perception nodes in the container ---
    lifecycle_node_names = [
        'image_undistort_node',
        'segmentation_inference_onnx_node',
        'camera_lidar_fusion_node',
        'occupancy_grid_builder_node',
    ]
    lifecycle_manager_backbone = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_perception',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': lifecycle_node_names,
            'bond_timeout': 0.0,
        }],
    )

    return LaunchDescription([
        topic_arg,
        visualize_arg,
        use_sim_time_arg,
        base_frame_arg,
        lidar_odom_frame_arg,
        robot_state_publisher_node,
        rviz_node,
        map_to_odom_publish_transform,
        ekf_node,
        perception_container,
        lifecycle_manager_backbone,
    ])