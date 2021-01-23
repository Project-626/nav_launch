import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = bringup_dir = get_package_share_directory('nav_launch')
    nav_params = LaunchConfiguration('nav_params')
    bt = LaunchConfiguration('bt')
    urdf = LaunchConfiguration('urdf')
    cartographer_params = LaunchConfiguration('cartographer_params')

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator',
                       'waypoint_follower']

    param_substitutions = {
        'default_bt_xml_filename': bt,
        }

    configured_nav_params = RewrittenYaml(
            source_file=nav_params,
            param_rewrites=param_substitutions,
            convert_types=True)

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument(
            'nav_params', default_value=os.path.join(share, 'parameters.yaml'),
            description='configuration file for navigation'),
        DeclareLaunchArgument(
            'bt', default_value=os.path.join(share, 'navigation.bt.xml'),
            description='behavior tree for navigation'),
        DeclareLaunchArgument(
            'urdf', default_value=os.path.join(share, 'p626.urdf.xml'),
            description='urdf file'),
        DeclareLaunchArgument(
            'cartographer_params', default_value='cartographer_2d.lua',
            description='urdf file'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[urdf]),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=['-configuration_directory', share, '-configuration_basename', cartographer_params],
            remappings=[('echoes', 'scan')],
        ),
        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            name='occupancy_grid_node',
            arguments=['resolution', '0.05'],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_nav_params],
            ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_nav_params],
            ),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[configured_nav_params],
            ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_nav_params],
            ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_nav_params],
            ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}]),

    ])
