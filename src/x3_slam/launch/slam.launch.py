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

    slam_launch_path = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    )

    slam_config_path = PathJoinSubstitution(
        [FindPackageShare('x3_slam'), 'config', 'slam.yaml']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('x3_slam'), 'config', 'robot_slam.rviz']
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

    lc = LaunchContext()
    ros_distro = EnvironmentVariable('ROS_DISTRO')
    slam_param_name = 'slam_params_file'
    if ros_distro.perform(lc) == 'foxy':
        slam_param_name = 'params_file'

    use_gmapping = input("Use Gmapping? (true or false): ")

    if "true" in use_gmapping or "True" in use_gmapping:
        pass
    elif "false" in use_gmapping or "False" in use_gmapping:
        pass
    else:
        print('please select either true or false')

    return LaunchDescription([

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Enable the use_sim_time parameter for Gazebo applications'
        ),

        DeclareLaunchArgument(
            name='use_gmapping',
            default_value=use_gmapping,
            description='Whether to use gmapping or slam_toolbox - default is false'
        ),

        DeclareLaunchArgument(
            name='rviz',
            default_value='false',
            description='Enable RVIZ'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_path)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration("use_sim_time"),
                slam_param_name: slam_config_path
            }.items(),
            condition = LaunchConfigurationEquals('use_gmapping','false')
        ),

        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            name='slam_gmapping_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("slam_gmapping"), "params", "slam_gmapping.yaml")],
            remappings = [("/scan","/downsampled_scan")],
            condition = LaunchConfigurationEquals('use_gmapping','true')
        ),

        Node(
            package="x3_slam",
            executable="scan_filter.py",
            name='scan_filter_node',
            condition = LaunchConfigurationEquals('use_gmapping','true')
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}]
        ),
    
        lidar_node,
        tf_base_link_to_laser

    ])