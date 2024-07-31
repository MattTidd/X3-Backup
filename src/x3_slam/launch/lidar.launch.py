import os
from launch import LaunchDescription
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    bringup_path = PathJoinSubstitution(
        [FindPackageShare('x3_bringup'), 'launch','x3_bringup_launch.py']
    )

    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rplidar_ros'), 'launch'),
            '/rplidar_s2_launch.py'])
    )

    tf_base_link_to_laser = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_publisher_node',
            arguments = ['0.0435', '5.258E-05', '0.11', '3.14', '0', '0', 'base_link', 'laser'],
    )

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_path)
        ),

        Node(
            package="x3_slam",
            executable="scan_filter.py",
            name='scan_filter_node',
        ),

        lidar_node,
        tf_base_link_to_laser
    ])
