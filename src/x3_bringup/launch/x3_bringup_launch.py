# Copyright (c) 2024 Matthew Allan Tidd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

"""
what am I launching?
- robot_state_publisher
- joint_state_publisher
- x3_driver
- x3_base_node
- imu_filter_madgwick
- joystick_handler
- ekf

"""

def generate_launch_description():

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("x3_description"), 'urdf', 'x3.urdf.xacro']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('x3_description'), 'config', 'robot.rviz']
    )

    imu_filter_config = os.path.join(              
        get_package_share_directory('x3_bringup'),
        'param',
        'imu_filter_param.yaml'
    ) 

    ekf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robot_localization'), 'launch'),
            '/ekf_x1_x3_launch.py'])
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='urdf', 
            default_value=urdf_path,
            description='URDF path'
        ),
        
        DeclareLaunchArgument(
            name='publish_joints', 
            default_value='true',
            description='Launch joint_states_publisher'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),

        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            name='pub_odom_tf',
            default_value='false',
            description='Whether to publish the tf from the original odom to the base_footprint'
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher_node',
            condition=IfCondition(LaunchConfiguration("publish_joints")),
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_node',
            output='screen',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'robot_description': Command(['xacro ', LaunchConfiguration('urdf')])
                }
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        Node(
            package='x3_bringup',
            executable='x3_driver',
            name='driver_node',
        ),

        Node(
            package='x3_base_node',
            executable='base_node_X3',
            name='base_node',
            parameters=[{'pub_odom_tf': LaunchConfiguration('pub_odom_tf')}]
        ),

        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            parameters=[imu_filter_config]
        ),

        Node(
            package='x3_control',
            executable='x3_joy',
            name='x3_joy_node'
        ),

       ekf_node

    ])
