import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # use sim time 	# param1
    use_sim_time = LaunchConfiguration('use_sim_time')		

    # URDF 경로		# param2
    pkg_path = os.path.join(get_package_share_directory('ros2_gazebo'))
    xacro_file = os.path.join(pkg_path,'description','ros2_gazebo.urdf.xacro')	
    robot_description_config = xacro.process_file(xacro_file)	
    
    # param
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    
    # rsp node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]	# 2 params
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher
    ])
