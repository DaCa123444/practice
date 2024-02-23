## Robot Package practice.


		
# branch Lv1. rob_description (rsp)
		rob_description xml ( xacro -> xml)
		rsp; node				: param - 1. rob_description.xacro 2. sim_time
		/joint_state; sub obj	 : auto (by rsp.cpp)
		/rob_description; pub obj	: auto (rsp.cpp)

		/img ; ps 
		/src ; rsp.cpp
		
		/ world로 고정 프레임 변경해줘야함.
		// urdf 기술시 joint로 연결할때, joint에서 pose를 선정해줘야함. 공중에 뜨지 않도록 주의
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
		gazebo plugin /gazebo_ros_ray_sensor(lv3).cpp ; gz msgs 구독하여 ros2로 송출 (  lazy node )
# branch Lv4. cam. 

		rsp
		gz_ros
		linear lidar
		camera (using rviz2)
		
		gazebo_ros_camera(lv4).cpp // 여러 차의 데이터를 동시 처리 가능한 형태로 제작되었으며,
									// odom 메시지로 현재 차량 상태 객체를 생성하고 이를 메시지화하여 ros2에서 pub 함 

				//odom base ? ; odom : 주행 거리 측정 메시지
		```
			  // Book: Sigwart 2011 Autonompus Mobile Robots page:337
			  // 이동 거리를 계산하여 로봇의 운동 상태를 추정
			  
			  double sl = vl * (wheel_diameter_[0] / 2.0) * seconds_since_last_update;
			  double sr = vr * (wheel_diameter_[0] / 2.0) * seconds_since_last_update;
			  double ssum = sl + sr; // 합 : 직선 거리

			  double sdiff = sr - sl; // 차 : 회전 거리
		```
		키 조작 안되던 문제 해결

	해결 : 원인 : 앞서 robot.urdf.xacro에서 gazebo_control을 주석처리
				+ 동시에 구별하기 위해 robot1.urdf.xacro를 rsp.launch.py로 실행하여 문제 발생
	 기존 방식으로 처리 
	 	config : x
	 	description : gazebo_control.xacro-  `<plugin name='diff_drive' filename='libgazebo_ros_diff_drive.so'`
	 				inertial_macros.xacro
	 				robot.urdf.xacro
	 				robot_core.xacro
	 				
	 	launch : launch_sim.launch.py : rsp + gzebo +spawn_entity
	 	
	 	
	 	teleop_twist_keyboard : /cmd_vel -> gazebo_control.xacro의 plugin diff_drive파일에 구독하는 부분이 있음

		```
		
			// cmd_vel 구독 생성
		  impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
			"cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
			std::bind(&GazeboRosDiffDrivePrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

		```
		
		위 코드분석을 통해, 여기서 전달된 값에 바퀴 속도가 gazebo로 업데이트되는 것이 아니라 로봇 본체가 전달되는 것	/tf를 직접 구현하여 /joint_state를 제외해버리는 것
		
		
		따라서 메시지로 구현하고 /joint_states를 생성하여 표현하기로함 ( ros2_humble인가 튜토리얼있음)
		
		실패함.
		
		gazebo 메시지 형식 및 모델 정보에 접근할 수가 없어 진전이 없다.
		
		gz client 내부에서 만져봐도 내부 물리적용만 작용한다.
		
		이에 다른 방법은 open_manip를 재구성하는 것이나, 유기적이기 때문에 오랜 시간이 걸린다. 일단 보류한다.
		
# branch Lv5. gz_control / gz를 위한 인터페이스 적용
	real : /cmd_vel -> ctrl sys -> motor driver(arduino)
	
	sim : /cmd_vel -> gz -> sim_robot
	
	----
	 - gazebo_control.xacro - plugin `libgazebo_ros_diff_drive.so` 로도 작성할 수 있겠으나, 매니퓰레이터를 위한 드라이버 생성(기존 파일 살짝 가공)
	 - pub `/cmd_vel`
	 
	 
	 - teleop_key ~ & ign_ros_bridge pkg
	 
	 - 컨트롤 구성 파악하고 다시 생성할 필요가 있음
	 - odom 프레임과 차체 프레임에 대해 개념 필요
# branch Lv6. ros2_control / 인터페이스, 추상화, 컨트롤 노드
	역할 : 하드웨어 장비를 그 인터페이스(드라이버에서 읽음)로 제어하기 위한 표준화 시스템
		1. 컨트롤러 관리자 : 드라이버와 컨트롤러 연결
		2. 하드웨어 인터페이스 : 각 장비에 맞게 다른 구성을 갖음	- 아두이노에 드라이버 구현
		3. 명령/상태 인터페이스 : 하드웨어에 전달하고 받아올 인터페이스 - ros2 구현
		4. 컨트럴러 : 데이터를 주고 받는 역할 ( 큰 영향력 없음)( 명령/상태 인터페이스 형태 구현)
		
		* 컨트롤러관리자 실행 - 	1. ors2_control_node pkg의 controller_manager 를 실행
								2. @자체 노드 작성하고 컨트롤러 관리자를 인스턴스화@
							- 둘 다, 하드웨어 인터페이스(urdf)와 컨트롤러 정보(yaml) 제공해야함
		리소스 관리자, 컨트롤러 관리자 - urdf plugin찾아보기
		
		ros2 control - 
		
# branch Lv7. opencv &  face detect

# branch Lv8. moveit2 + ompl

# branch slv0. turtlebot + manip 의 경우도 살펴보기

# branch sLv1. slam (지도)

# branch sLv2. navigation 

#s3 mobile control ( app )
