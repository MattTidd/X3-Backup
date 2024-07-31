from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    map_path = PathJoinSubstitution(
        [FindPackageShare('x3_nav'), 'maps', 'x3.yaml']
    )

    nav2_param_path = PathJoinSubstitution(
        [FindPackageShare('x3_nav'), 'params', 'mppi_nav_params.yaml']
    )

    nav2_bringup_path = PathJoinSubstitution([FindPackageShare('nav2_bringup')])

    return LaunchDescription([

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Enable the use_sim_time parameter'
        ),

        DeclareLaunchArgument(
            name='map',
            default_value=map_path,
            description='Path to the map file to be used in navigation'
        ),

        DeclareLaunchArgument(
            name='params_file',
            default_value=nav2_param_path,
            description='Path to the parameters for MPPI navigation'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_path, '/launch', '/bringup_launch.py']),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('params_file')}.items(),
        ),
    ])
