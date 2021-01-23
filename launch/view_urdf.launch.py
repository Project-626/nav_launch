import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'p626.urdf.xml'

    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
    get_package_share_directory('nav_launch'),
    'urdf/',urdf_file_name)
    rviz_config = os.path.join(
    get_package_share_directory('nav_launch'),
    'configs/view_config.rviz')
    
    if os.path.exists(rviz_config) != 1:
        print("Can't find Rviz configuration file")
    else:
        print(rviz_config)
	
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]),
    	Node(
    	    package='rviz2',
    	    executable='rviz2',
    	    name='Rviz2',
    	    output='screen',
    	    arguments=['-d'+rviz_config])        
    ])
