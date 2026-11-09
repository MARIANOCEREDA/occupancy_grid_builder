from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch.conditions import IfCondition
from launch.substitutions import NotEqualsSubstitution
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory
import os

import xacro


def generate_launch_description():

    params_file = os.path.join(
        get_package_share_directory('occupancy_grid_builder_bringup'),
        'config',
        'params.yaml'
    )

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
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='',
        description='Rosbag to play inside the perception container (empty = no playback)'
    )
    bag_start_paused_arg = DeclareLaunchArgument(
        'bag_start_paused',
        default_value='false',
        description='Start the bag paused; resume with the /rosbag2_player/resume service'
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
        parameters=[params_file, {
            'base_frame': LaunchConfiguration('base_frame'),
            'lidar_odom_frame': LaunchConfiguration('lidar_odom_frame'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
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

    # --- Ground Segmentation Component ---
    ground_segmentation_component = ComposableNode(
        package='ground_segmentation',
        plugin='ground_segmentation::GroundSegmentationNode',
        name='ground_segmentation_node',
        parameters=[params_file],
    )

    # --- Image undistort Component ---
    image_undistort_component = ComposableNode(
        package='image_processing',
        plugin='image_undistort::ImageUndistort',
        name='image_undistort_node',
        parameters=[params_file, {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # --- Segmentation Inference ONNX Component ---
    segmentation_inference_onnx_component = ComposableNode(
        package='yolo_onnx_ros',
        plugin='yolo_onnx_ros::YoloSegmentorNode',
        name='yolo_onnx_segmentor_node',
        parameters=[params_file, {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'model_path': os.path.join(
                get_package_share_directory('occupancy_grid_builder_bringup'),
                'models',
                'yolov11n-seg.onnx'
            ),
        }],
    )

    # --- PCL Labeler Component ---
    pcl_labeler_component = ComposableNode(
        package='pcl_labeler',
        plugin='pcl_labeler::PCLLabelerNode',
        name='pcl_labeler_node',
        parameters=[params_file, {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # --- Occupancy Grid Builder Component ---
    occupancy_grid_builder_component = ComposableNode(
        package='occupancy_grid_builder',
        plugin='occupancy_grid_builder::OccupancyGridBuilderNode',
        name='occupancy_grid_builder_node',
        parameters=[
            params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    # --- EKF node ---
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[params_file,
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
    local_costmap_node = LifecycleNode(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        namespace='local_costmap',
        name='local_costmap',
        output='screen',
        parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    perception_container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            kiss_icp_component,
            ground_segmentation_component,
            image_undistort_component,
            segmentation_inference_onnx_component,
            pcl_labeler_component,
            occupancy_grid_builder_component
        ],
        output='screen',
    )

    # --- Rosbag player (publishes /clock, so it must not use sim time itself) ---
    bag_player_component = ComposableNode(
        package='rosbag2_transport',
        plugin='rosbag2_transport::Player',
        name='rosbag2_player',
        parameters=[{
            'storage.uri': ParameterValue(LaunchConfiguration('bag_path'), value_type=str),
            'play.clock_publish_frequency': 100.0,
            'play.start_paused': ParameterValue(
                LaunchConfiguration('bag_start_paused'), value_type=bool),
            'play.disable_keyboard_controls': True,
        }],
    )

    load_bag_player = LoadComposableNodes(
        target_container='/perception_container',
        composable_node_descriptions=[bag_player_component],
        condition=IfCondition(NotEqualsSubstitution(LaunchConfiguration('bag_path'), '')),
    )

    # --- Lifecycle manager for all perception nodes in the container ---
    lifecycle_node_names = [
        'image_undistort_node',
        'yolo_onnx_segmentor_node',
        'pcl_labeler_node',
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
        bag_path_arg,
        bag_start_paused_arg,
        robot_state_publisher_node,
        rviz_node,
        map_to_odom_publish_transform,
        ekf_node,
        perception_container,
        load_bag_player,
        lifecycle_manager_backbone,
    ])