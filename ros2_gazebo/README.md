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

핵심 :	궤적의 이동은 아니고, ros2 topic을 gazebo에서 받아서 커스텀된 모델의 데이터 값을 업데이트할 수 있다.
  	ros2_control.xacro는 gz_system임을 밝힌다. // 시뮬이 아닐경우 커스텀된 하드웨어 인터페이스를 플러그인으로 사용할 것같다
  	
  	gz_control.xacro는 플러그인을 호출한다. ("libgazebo_ros2_control.so" // 미리 컴파일되어 공유라이브러리에서사용)
  										이는 모듈형으로 커스텀 모델에 맞게 yaml파일로 인터페이스 형태를 제공해야한다.
  		*이 내용은 그대로 gazebo에서 실행되며 메시지 유형등을 결정
  	
  	gazebo_ros2_control과   ros2_control의 차이점 : 아직 ros2_control은 학습하지 않았으나, control_node(controller_manager)를 생성하지 않았음에도 파라미터 설정을 통해 해당 노드가 생성됨을 확인하였다.
  	
  	
  		'''
		  		ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
		- 0.0
		- 0.0
		- 0.0"
		'''
  		
			*참조 rrbot ; ros2_control_example_9  


gazebo_ros_control에 대해.
1. urdf.xacro 
				- 이걸 원칙으로 삼아 다시 제작하자.
				- robot에 대해 정의 + 가제보 태그는 색상만 정의
				- 추가로, ros-control을 정의 (인터페이스 유형, 조인트 이름과 하드인터페이스, 액츄에이터 인터페이스 정의)
				- to do. urdf.xacro 파일에 ros-control 정의 추가하기(transmission)
				
	articuone bot에서 gazebo_control의 경우  /joint_states를 업데이트하지 않는다.
	대신 robot_states_pub이 출력하는 tf를 같이 출력해서 업데이트된 결과를 덮어쓴다.
	
	아래는 해당 내용의 gazebo_ros_diff_drive(lv5).cpp파일의 일부이다.
	
<!-- 
  // Create TF broadcaster if needed
  impl_->publish_wheel_tf_ = _sdf->Get<bool>("publish_wheel_tf", false).first;
  impl_->publish_odom_tf_ = _sdf->Get<bool>("publish_odom_tf", false).first;
  if (impl_->publish_wheel_tf_ || impl_->publish_odom_tf_) {
    // tf 브로드 캐스터 생성; ros에 tf msg 발행
    impl_->transform_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);

	// tf가 true이면, odom 프레임과 로봇 베이스 프레임간  tf 변환을 로그 출력
    if (impl_->publish_odom_tf_) {
      RCLCPP_INFO(
        impl_->ros_node_->get_logger(),
        "Publishing odom transforms between [%s] and [%s]", impl_->odometry_frame_.c_str(),
        impl_->robot_base_frame_.c_str());
    }
	// 휠 쌍에 대한 TF 변환을 발행하는 로그 출력
    for (index = 0; index < impl_->num_wheel_pairs_; ++index) {
      if (impl_->publish_wheel_tf_) {
        RCLCPP_INFO(
          impl_->ros_node_->get_logger(),
          "Publishing wheel transforms between [%s], [%s] and [%s]",
          impl_->robot_base_frame_.c_str(),
          impl_->joints_[2 * index + GazeboRosDiffDrivePrivate::LEFT]->GetName().c_str(),
          impl_->joints_[2 * index + GazeboRosDiffDrivePrivate::RIGHT]->GetName().c_str());
      }
    }
  }

-->


camera.xacro 의 오류 수정
- 우측 90도로 회전되어 나타나는 양상을 정상으로 수정하였음. 원리 파악 필요

    <joint name="camera_joint" type="fixed">
        <parent link="joint_link3"/>
        <child link="camera_link"/>
        <origin xyz="0 0 0" rpy="${-pi/2} 0 0"/>
    </joint>

  	
  	
  			
	역할 : 하드웨어 장비를 그 인터페이스(드라이버에서 읽음)로 제어하기 위한 표준화 시스템
		1. 컨트롤러 관리자 : 드라이버와 컨트롤러 연결
		2. 하드웨어 인터페이스 : 각 장비에 맞게 다른 구성을 갖음	- 아두이노에 드라이버 구현
		3. 명령/상태 인터페이스 : 하드웨어에 전달하고 받아올 인터페이스 - ros2 구현
		4. 컨트럴러 : 데이터를 주고 받는 역할 ( 큰 영향력 없음)( 명령/상태 인터페이스 형태 구현)
		
		* 컨트롤러관리자 실행 - 	1. ors2_control_node pkg의 controller_manager 를 실행
								2. @자체 노드 작성하고 컨트롤러 관리자를 인스턴스화@
							- 둘 다, 하드웨어 인터페이스(urdf)와 컨트롤러 정보(yaml) 제공해야함
		리소스 관리자, 컨트롤러 관리자 - urdf plugin찾아보기
		
		ros2 control   vs gz_ros2_control
			1. ros2_control	: 	조인트를 사용하기 위해 매개변수를 정의.
								joint + param + command_interface:state_interface"
				
				
			2. gz_ros2_control : ros2_control 태그를 실제 분석, 적절한 인터페이스,컨트롤러 관리자 로드
								ros2_control과 gz 사이 커스텀 로봇 하드웨어 인터페이스 생성 가능
								
				1은 gz와 ros를 연결하고 인터페이스를 제공.
				2는 이 제공된 컨트롤러 관리 및 커스텀 인터페이스를 제공
				
			3. transmission tag :   조인트를 액츄에이터에 연결
		```
			<transmission name="tran1">
				<type>transmission_interface/SimpleTransmission</type>
				<joint name="shoulder_pan_joint">
				    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
				</joint>
				<actuator name="motor1">
				    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
				    <mechanicalReduction>1</mechanicalReduction>
				</actuator>
			</transmission>
  		```
  		
  		teleop_twist_keyboard를 통해 작동하고 싶었으나,원하는 데이터를 전송하는 라이브러리가 없어 진행하지 못함.
  		대신 컨트롤러를 추가하고, /joint_states를 gz로도 연결되도록 함.
  		
# branch Lv6. ros2_control / 인터페이스, 추상화, 컨트롤 노드


  		
		
# branch Lv7. opencv &  face detect

# branch Lv8. moveit2 + ompl

# branch slv0. turtlebot + manip 의 경우도 살펴보기

# branch sLv1. slam (지도)

# branch sLv2. navigation 

#s3 mobile control ( app )
