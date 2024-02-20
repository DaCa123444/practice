## Robot Package practice.


		
# branch Lv1. rob_description (rsp)
		rob_description xml ( xacro -> xml)
		rsp; node				: param - 1. rob_description.xacro 2. sim_time
		/joint_state; sub obj	 : auto (by rsp.cpp)
		/rob_description; pub obj	: auto (rsp.cpp)

		/img ; ps 
		/src ; rsp.cpp
		
		/ world와 관계로 설정해놨기 때문에 fixed frame을 바꾸지 않아도 제대로 볼 수 있음.
		단 관성등을 추가해야함 - lv2에서도 문제 발생하여 보이지 않음( 추가해도 보이지 않았음 - spawner를 통해 토픽을 전달하지 않았음)
# branch Lv2. rsp & gazebo
		
		rsp launch
		gz_ros gzserver launch
		
		spawn_entity
		
		gzserver : bin file  // + plugin  ros_init , ros_factory, ros_force
		
		+spawner ; gz_ros pkg의 spawn_entity.py 로 urdf를 전달하는 노드

		error 1. tuple ; comma 문제
		
		
		
# branch Lv3. linear lidar 
		
		rsp
		gz_ros
		linear ( in rob_description, add laser link & gazebo ) // topic : sensor_msgs/Laserscan

		--
		ros2 service list - /spawn_entity
# branch Lv4. cam. 

# branch Lv5. opencv & face detect. 

# branch Lv6. ros2_control 실제 제어 -인터페이스interface,Abstraction,control node?)


# branch sLv1. slam (지도)

# branch sLv2. navigation 

#s3 mobile control ( app )
