"""Sentinel Swarm — Phase 1 single-drone Offboard bringup.

This brings up the Offboard waypoint controller, and optionally MAVROS. PX4
SITL and Gazebo run as separate external processes (see README), since they are
not ROS 2 packages.

Typical sequence:
  # terminal 1 — PX4 SITL + Gazebo
  cd ~/PX4-Autopilot && make px4_sitl gz_x500

  # terminal 2 — this launch (also starts MAVROS)
  ros2 launch drone_bringup drone.launch.py mavros:=true

Tune the controller live, e.g.:
  ros2 launch drone_bringup drone.launch.py mavros:=true kp_xy:=1.2 kd_xy:=0.25
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def gain(name):
    """Resolve a launch-argument gain to a typed float parameter."""
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = LaunchConfiguration('gcs_url')
    use_mavros = LaunchConfiguration('mavros')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url', default_value='udp://:14540@127.0.0.1:14557',
            description='MAVLink endpoint for PX4 SITL'),
        DeclareLaunchArgument(
            'gcs_url', default_value='',
            description='optional ground-station bridge URL'),
        DeclareLaunchArgument(
            'mavros', default_value='false',
            description='also launch the MAVROS node (true/false)'),
        DeclareLaunchArgument('kp_xy', default_value='0.9'),
        DeclareLaunchArgument('ki_xy', default_value='0.05'),
        DeclareLaunchArgument('kd_xy', default_value='0.15'),
        DeclareLaunchArgument('kp_z', default_value='1.2'),
        DeclareLaunchArgument('ki_z', default_value='0.1'),
        DeclareLaunchArgument('kd_z', default_value='0.0'),

        Node(
            package='mavros', executable='mavros_node', name='mavros',
            output='screen',
            condition=IfCondition(use_mavros),
            parameters=[{
                'fcu_url': fcu_url,
                'gcs_url': gcs_url,
            }],
        ),

        Node(
            package='drone_offboard', executable='offboard_node',
            name='drone_offboard', output='screen',
            parameters=[{
                'kp_xy': gain('kp_xy'),
                'ki_xy': gain('ki_xy'),
                'kd_xy': gain('kd_xy'),
                'kp_z': gain('kp_z'),
                'ki_z': gain('ki_z'),
                'kd_z': gain('kd_z'),
            }],
        ),
    ])
