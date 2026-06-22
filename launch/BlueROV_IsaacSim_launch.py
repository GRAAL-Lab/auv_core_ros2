from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # --------------------------------------------------
    # Launch Arguments
    # --------------------------------------------------
    config_name_arg = DeclareLaunchArgument(
        'config_name',
        default_value='BlueROV',
        description='Configuration name to load parameters'
    )

    sim_params_file_arg = DeclareLaunchArgument(
        'sim_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('sim'),
            'param',
            'params.yaml'
        ]),
        description='Path to simulator parameters YAML file'
    )


    # --------------------------------------------------
    # Launch Configurations
    # --------------------------------------------------
    config_name = LaunchConfiguration('config_name')
    sim_params_file = LaunchConfiguration('sim_params_file')

    # --------------------------------------------------
    # Launch Description
    # --------------------------------------------------
    return LaunchDescription([

        # Declare arguments
        config_name_arg,
        sim_params_file_arg,

        # --------------------------------------------------
        # Joy Node (no params)
        # --------------------------------------------------
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # --------------------------------------------------
        # Interface Node (new terminal)
        # --------------------------------------------------
        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'gnome-terminal',
                        '--',
                        'bash',
                        '-c',
                        'ros2 run kcl interface_node; exec bash'
                    ],
                    output='screen'
                )
            ]
        ),

        # --------------------------------------------------
        # Kinematic Control Layer (SIM params)
        # --------------------------------------------------
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='kcl',
                    executable='kinematic_control_layer_node',
                    name='kcl',
                    output='screen',
                    parameters=[
                        sim_params_file,
                        {'config_name': config_name}
                    ]
                )
            ]
        ),

        # --------------------------------------------------
        # Dynamic Control Layer (SIM params)
        # --------------------------------------------------
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='dcl',
                    executable='dynamic_control_layer_node',
                    name='dcl',
                    output='screen',
                    parameters=[
                        sim_params_file,
                        {'config_name': config_name}
                    ]
                )
            ]
        ),

        # --------------------------------------------------
        # Visualizer Node (SIM params)
        # --------------------------------------------------
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='viz',
                    executable='visualizer_node',
                    name='visualizer',
                    output='screen',
                    parameters=[
                        sim_params_file,
                        {'config_name': config_name}
                    ]
                )
            ]
        ),
    ])
