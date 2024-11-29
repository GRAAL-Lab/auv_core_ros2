from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the launch argument
    config_name_arg = DeclareLaunchArgument(
        'config_name',
        default_value='x300',
        description='Configuration name to load parameters'
    )

    # Get the launch configuration
    config_name = LaunchConfiguration('config_name')

    return LaunchDescription([
        # Declare the launch argument
        config_name_arg,

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        # Replace running the interface using konsole with a new terminal
        TimerAction(
            period=1.0,  # Wait 1 second before starting the next node
            actions=[
                ExecuteProcess(
                    cmd=[
                        'gnome-terminal -- bash -c "ros2 run kcl interface_node"'
                    ],
                    output='screen',
                    shell=True
                )
            ]
        ),
        TimerAction(
            period=1.0,  # Wait 1 second before starting the next node
            actions=[
                Node(
                    package='kcl',
                    executable='kinematic_control_layer_node',
                    name='kcl',
                    output='screen',
                    parameters=[{'config_name': config_name}]  # Pass the launch argument
                )
            ]
        ),
        TimerAction(
            period=1.0,  # Wait another 1 second before starting the next node
            actions=[
                Node(
                    package='dcl',
                    executable='dynamic_control_layer_node',
                    name='dcl',
                    output='screen',
                    parameters=[{'config_name': config_name}]  # Pass the launch argument
                )
            ]
        ),
        TimerAction(
            period=1.0,  # Wait another 1 seconds before starting the next node
            actions=[
                Node(
                    package='sim',
                    executable='simulator_node',
                    name='simulator',
                    output='screen',
                    parameters=[{'config_name': config_name}]  # Pass the launch argument
                )
            ]
        ),
        TimerAction(
            period=1.0,  # Wait another 1 second before starting the next node
            actions=[
                Node(
                    package='viz',
                    executable='visualizer_node',
                    name='visualizer',
                    output='screen',
                    parameters=[{'config_name': config_name}]  # Pass the launch argument
                )
            ]
        )
    ])
