# navigation.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get directories
    go1_nav_dir = get_package_share_directory('go1_navigation')

    # Launch configurations
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    # Lifecycle nodes
    lifecycle_nodes = ['planner_server', 'controller_server', 'bt_navigator']

    # Declare launch arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(go1_nav_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the navigation stack'
    )
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for nodes'
    )

    # Rewritten YAML
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'robot_base_frame': 'base_link'
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # Nodes
    nodes = [
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=[('cmd_vel', '/cmd_vel_nav')],
            arguments=['--ros-args', '--log-level', log_level]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            output='screen',
            parameters=[configured_params],
            remappings=[('cmd_vel', '/cmd_vel')],
            arguments=['--ros-args', '--log-level', log_level]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            output='screen',
            parameters=[configured_params],
            remappings=[('cmd_vel', '/cmd_vel')],
            arguments=['--ros-args', '--log-level', log_level]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes
            }],
            arguments=['--ros-args', '--log-level', log_level]
        ),
    ]

    # Launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_log_level_cmd)

    # Set environment variable
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    # Add nodes
    for node in nodes:
        ld.add_action(node)

    return ld

