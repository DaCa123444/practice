## Robot Package practice.


		
# branch Lv1. rob_description (rsp)
		rob_description xml ( xacro -> xml)
		rsp; node				: param - 1. rob_description.xacro 2. sim_time
		/joint_state; sub obj	 : auto (by rsp.cpp)
		/rob_description; pub obj	: auto (rsp.cpp)

		/img ; ps 
		/src ; rsp.cpp
# branch Lv2. rsp & gazebo
		
		rsp launch
		gz_ros gzserver launch
		
		spawn_entity
		
		gzserver : bin file  // + plugin  ros_init , ros_factory, ros_force

# branch Lv3. linear lidar 
		
		rsp
		gz_ros
		linear

# branch Lv4. cam. 

# branch Lv5. opencv & face detect. 

# branch Lv6. ros2_control 실제 제어 -인터페이스interface,Abstraction,control node?)


# branch sLv1. slam (지도)

# branch sLv2. navigation 

#s3 mobile control ( app )
