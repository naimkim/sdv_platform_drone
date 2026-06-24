"""Sentinel Swarm — Phase 2 GPS-denied navigation bringup.

Brings up the localization node (VIO/GPS fusion + GPS integrity monitor) and the
reactive avoidance node. The VIO odometry (/vio/odom), GPS fixes (/gps/pose) and
LaserScan (/scan) come from PX4 SITL + Gazebo (or a sim/bag) — see README.

Examples:
  ros2 launch drone_bringup phase2_navigation.launch.py
  ros2 launch drone_bringup phase2_navigation.launch.py goal_x:=8.0 goal_y:=2.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def coord(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('goal_x', default_value='5.0'),
        DeclareLaunchArgument('goal_y', default_value='5.0'),

        Node(
            package='drone_localization', executable='localization_node',
            name='drone_localization', output='screen'),

        Node(
            package='drone_avoidance', executable='avoidance_node',
            name='drone_avoidance', output='screen',
            parameters=[{
                'goal_x': coord('goal_x'),
                'goal_y': coord('goal_y'),
            }]),
    ])
