"""Sentinel Swarm — Phase 3 perception-aware navigation bringup.

Includes the Phase 2 navigation stack (localization + avoidance) and adds the
perception bridge, which turns upstream YOLO detections (vision_msgs) into world
obstacles for the avoidance planner.

The detector itself (YOLO + TensorRT on Jetson) is an upstream node publishing
vision_msgs/Detection2DArray on /detections — see README.

Examples:
  ros2 launch drone_bringup phase3_perception.launch.py
  ros2 launch drone_bringup phase3_perception.launch.py goal_x:=8.0 hfov_deg:=86.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('drone_bringup')
    phase2 = os.path.join(
        bringup_share, 'launch', 'phase2_navigation.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('goal_x', default_value='5.0'),
        DeclareLaunchArgument('goal_y', default_value='5.0'),
        DeclareLaunchArgument('image_width', default_value='640'),
        DeclareLaunchArgument('hfov_deg', default_value='90.0'),
        DeclareLaunchArgument('object_height_m', default_value='1.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(phase2),
            launch_arguments={
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
            }.items()),

        Node(
            package='drone_perception', executable='perception_node',
            name='drone_perception', output='screen',
            parameters=[{
                'image_width': ParameterValue(
                    LaunchConfiguration('image_width'), value_type=int),
                'hfov_deg': ParameterValue(
                    LaunchConfiguration('hfov_deg'), value_type=float),
                'object_height_m': ParameterValue(
                    LaunchConfiguration('object_height_m'), value_type=float),
            }]),
    ])
