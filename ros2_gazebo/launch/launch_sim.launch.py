import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler,ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.descriptions import ParameterValue

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
# urdf error 2월26일

#https://github.com/ros-controls/ros2_control/pull/1410
def generate_launch_description():

    package_name='ros2_gazebo' 

	#rob state pub
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # gzserver
    gzserver = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')])
             )
    
    
    #  gazebo에 모델 객체 생성함
    spawn_robot = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                              '-entity', 'robot'],
                    output='screen'
                    )



    # Get URDF via xacro
    robot_description_content =Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ros2_gazebo"),
                    "description",
                    "ros2_gazebo.urdf.xacro",
                ]
            ),
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ros2_gazebo"),
            "config",
            "gz1_controllers.yaml",                 
        ]
    )
    

#    control_node = Node(
#        package="controller_manager",
#        executable="ros2_control_node",
#        parameters=[robot_description, robot_controllers], #urdf error-1원래는 parameters=[robot_description, robot_controllers]
#        output="both",
#    )
#    robot_state_pub_node = Node(
#        package="robot_state_publisher",
#        executable="robot_state_publisher",
#        output="both",
#        parameters=[robot_description]  #urdf error-2원래는 parameters=[robot_description]
#    )


		# 컨트롤러 관련 부분 - 1. js에 대해 방송
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster","--controller-manager", "/controller_manager"],
    )

		# 컨트롤러 - forward_position_control  -> joint_position_controller
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],  # forward_position_controller #gz_controllers.yaml
    )

           #arguments=["joint_position_controller", "--controller-manager", "/controller_manager"],

#    load_joint_state_controller = ExecuteProcess(
#        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
#             'joint_state_broadcaster'],
#        output='screen'
#    )

#    load_trajectory_controller = ExecuteProcess(
#        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
#             'joint_trajectory_controller'],
#        output='screen'
#    )



    # Launch
    return LaunchDescription([
        
        rsp,
        gzserver,
        spawn_robot,
        #control_node,
        #robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
    ])
    return LaunchDescription(declared_arguments + nodes)
    
    
    

#    ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "header:
#  stamp:
#    sec: 0
#    nanosec: 0
#  frame_id: ''
#joint_names: ['joint_1','joint_2','joint_4']
#points: [ {"positions":[-1.57,-0.59,-1.2]} ]"

