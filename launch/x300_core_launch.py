from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the launch arguments
    config_name_arg = DeclareLaunchArgument(
        'config_name',
        default_value='x300',
        description='Configuration name to load parameters'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('sim'),
            'param',
            'params.yaml'
        ]),
        description='Path to the parameters YAML file'
    )

    # Get the launch configuration values
    config_name = LaunchConfiguration('config_name')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        # Declare the launch arguments
        config_name_arg,
        params_file_arg,

        # joy_node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # Interface node in a new terminal
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

        # Kinematic Control Layer Node
        TimerAction(
            period=1.0,  # Wait 1 second before starting the next node
            actions=[
                Node(
                    package='kcl',
                    executable='kinematic_control_layer_node',
                    name='kcl',
                    output='screen',
                    parameters=[params_file, {'config_name': config_name}]  # Pass params file and config_name
                )
            ]
        ),

        # Dynamic Control Layer Node
        TimerAction(
            period=1.0,  # Wait 1 second before starting the next node
            actions=[
                Node(
                    package='dcl',
                    executable='dynamic_control_layer_node',
                    name='dcl',
                    output='screen',
                    parameters=[params_file, {'config_name': config_name}]  # Pass params file and config_name
                )
            ]
        ),

        # Simulator Node
        TimerAction(
            period=1.0,  # Wait 1 second before starting the next node
            actions=[
                Node(
                    package='sim',
                    executable='simulator_node',
                    name='simulator',
                    output='screen',
                    parameters=[params_file, {'config_name': config_name}]  # Pass params file and config_name
                )
            ]
        ),

        # Visualizer Node
        TimerAction(
            period=1.0,  # Wait 1 second before starting the next node
            actions=[
                Node(
                    package='viz',
                    executable='visualizer_node',
                    name='visualizer',
                    output='screen',
                    parameters=[params_file, {'config_name': config_name}]  # Pass params file and config_name
                )
            ]
        )
    ])

