"""Sentinel Swarm — Phase 4 multi-drone coordination bringup.

Launches an N-member swarm that partitions a square search area and flies
lawnmower coverage paths through it, sharing position over DDS and avoiding each
other — no attacker, no security overlay (that is Phase 5). Useful to watch the
coordination scale with the member count.

Examples:
  ros2 launch swarm_bringup swarm_coordination.launch.py
  ros2 launch swarm_bringup swarm_coordination.launch.py drones:=5 viz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SECRET = 'sentinel-swarm-group-key'


def launch_setup(context, *args, **kwargs):
    n = int(LaunchConfiguration('drones').perform(context))
    viz = LaunchConfiguration('viz').perform(context).lower() in (
        'true', '1', 'yes')

    nodes = []
    for i in range(n):
        drone_id = f'drone_{i + 1}'
        # Spread start points along the bottom edge so they don't overlap.
        start_x = (i + 0.5) * (10.0 / max(1, n))
        nodes.append(Node(
            package='swarm_agent', executable='agent_node', name=drone_id,
            parameters=[{
                'drone_id': drone_id,
                'secret': SECRET,
                'start_x': start_x,
                'start_y': 0.0,
            }],
            output='screen'))

    nodes.append(Node(
        package='swarm_consensus', executable='consensus_node',
        name='swarm_consensus',
        parameters=[{'quorum': 1, 'min_alerts': 3}], output='screen'))

    if viz:
        nodes.append(Node(
            package='swarm_viz', executable='viz_node', name='swarm_viz',
            output='screen'))
        rviz_config = os.path.join(
            get_package_share_directory('swarm_viz'),
            'rviz', 'sentinel_swarm.rviz')
        nodes.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_config], output='screen'))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'drones', default_value='4',
            description='number of swarm members to spawn'),
        DeclareLaunchArgument(
            'viz', default_value='false',
            description='launch swarm_viz + RViz (true/false)'),
        OpaqueFunction(function=launch_setup),
    ])
