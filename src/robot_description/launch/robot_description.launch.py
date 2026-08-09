"""Launch file for the robot description."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
import xacro



def generate_launch_description():

    visualize_arg = DeclareLaunchArgument(
        'visualize',
        default_value='false',
        description='Whether to visualize the robot in RViz'
    )

    pkg_name = 'robot_description'
    pkg_share = get_package_share_directory(pkg_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot_description.urdf.xacro')

    robot_description = xacro.process_file(urdf_file).toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'robot_description.rviz')],
        condition=IfCondition(LaunchConfiguration('visualize'))
    )

    return LaunchDescription([
        visualize_arg,
        robot_state_publisher_node,
        rviz_node,
    ])