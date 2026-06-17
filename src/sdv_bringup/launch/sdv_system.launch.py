from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def conditional_node(package, executable, launch_arg, parameters=None):
    return Node(
        package=package,
        executable=executable,
        name=executable,
        output='screen',
        condition=IfCondition(LaunchConfiguration(launch_arg)),
        parameters=parameters or [],
    )


def generate_launch_description():
    node_options = [
        ('vehicle_manager', 'vehicle_manager', 'vehicle_manager_node', 'true'),
        ('battery', 'battery_ecu', 'battery_node', 'true'),
        ('sensor', 'sensor_ecu', 'sensor_node', 'true'),
        ('motor', 'motor_ecu', 'motor_node', 'true'),
        ('diagnostics', 'diagnostics_ecu', 'diagnostics_node', 'true'),
        ('security', 'security_ecu', 'security_node', 'true'),
        ('gui', 'sdv_test_gui', 'test_gui_node', 'true'),
    ]

    actions = []
    for launch_arg, _package, _executable, default_value in node_options:
        actions.append(
            DeclareLaunchArgument(
                launch_arg,
                default_value=default_value,
                description=f'Start the {launch_arg} node.',
            )
        )

    actions.append(
        DeclareLaunchArgument(
            'sensor_driver_mode',
            default_value='sim',
            description='Sensor driver mode: sim or hw.',
        )
    )
    actions.append(
        DeclareLaunchArgument(
            'motor_driver_mode',
            default_value='sim',
            description='Motor driver mode: sim or hw.',
        )
    )

    for launch_arg, package, executable, _default_value in node_options:
        parameters = []
        if package == 'sensor_ecu':
            parameters = [
                {'driver_mode': LaunchConfiguration('sensor_driver_mode')}
            ]
        elif package == 'motor_ecu':
            parameters = [
                {'driver_mode': LaunchConfiguration('motor_driver_mode')}
            ]

        actions.append(
            conditional_node(package, executable, launch_arg, parameters)
        )

    return LaunchDescription(actions)
