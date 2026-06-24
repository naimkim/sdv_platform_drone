"""Sentinel Swarm — Phase 5 security layer bringup.

Launches a 3-member swarm with the IDS and the Byzantine consensus guard, and
optionally injects an attack. Pure ROS 2 / DDS — no PX4 or Gazebo required, so
the security story is demonstrable on its own.

Examples:
  ros2 launch swarm_bringup swarm_security.launch.py
  ros2 launch swarm_bringup swarm_security.launch.py attack:=outsider_spoof
  ros2 launch swarm_bringup swarm_security.launch.py attack:=replay
  ros2 launch swarm_bringup swarm_security.launch.py attack:=insider_teleport
  ros2 launch swarm_bringup swarm_security.launch.py attack:=insider_teleport viz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SECRET = 'sentinel-swarm-group-key'
START_POS = {
    'drone_1': (0.0, 0.0),
    'drone_2': (10.0, 0.0),
    'drone_3': (0.0, 10.0),
}


def honest_agent(drone_id):
    x, y = START_POS[drone_id]
    return Node(
        package='swarm_agent',
        executable='agent_node',
        name=drone_id,
        parameters=[{
            'drone_id': drone_id,
            'secret': SECRET,
            'start_x': x,
            'start_y': y,
        }],
        output='screen',
    )


def launch_setup(context, *args, **kwargs):
    attack = LaunchConfiguration('attack').perform(context)
    target = LaunchConfiguration('target').perform(context)
    viz = LaunchConfiguration('viz').perform(context).lower() in (
        'true', '1', 'yes')

    nodes = [honest_agent('drone_1'), honest_agent('drone_2')]

    # drone_3 is honest unless it is the compromised insider in the demo.
    if attack != 'insider_teleport':
        nodes.append(honest_agent('drone_3'))

    nodes.append(Node(
        package='swarm_ids', executable='ids_node', name='swarm_ids',
        parameters=[{'secret': SECRET}], output='screen'))

    nodes.append(Node(
        package='swarm_consensus', executable='consensus_node',
        name='swarm_consensus',
        parameters=[{'quorum': 1, 'min_alerts': 3}], output='screen'))

    if attack != 'none':
        attack_target = 'drone_3' if attack == 'insider_teleport' else target
        nodes.append(Node(
            package='swarm_attacker', executable='attacker_node',
            name='swarm_attacker',
            parameters=[{
                'attack_type': attack,
                'target_id': attack_target,
                'secret': SECRET,
            }],
            output='screen'))

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
            'attack', default_value='none',
            description='none | outsider_spoof | replay | insider_teleport'),
        DeclareLaunchArgument(
            'target', default_value='drone_2',
            description='victim drone_id for spoof/replay attacks'),
        DeclareLaunchArgument(
            'viz', default_value='false',
            description='launch swarm_viz + RViz (true/false)'),
        OpaqueFunction(function=launch_setup),
    ])
