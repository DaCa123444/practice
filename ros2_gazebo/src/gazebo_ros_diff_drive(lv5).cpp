// Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the company nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*
 * \file  gazebo_ros_diff_drive.cpp
 *
 * \brief A differential drive plugin for gazebo. Based on the diffdrive plugin
 * developed for the erratic robot (see copyright notice above). The original
 * plugin can be found in the ROS package gazebo_erratic_plugins.
 *
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * $ Id: 06/21/2013 11:23:40 AM piyushk $
 */

/*
 *
 * The support of acceleration limit was added by
 * \author   George Todoran <todorangrg@gmail.com>
 * \author   Markus Bader <markus.bader@tuwien.ac.at>
 * \date 22th of May 2014
 */
// Differential Drive"의 줄임말로, 주로 로봇 제어에서 사용되는 구조
// 이 구조는 주로 두 개의 바퀴가 있고, 이 두 바퀴를 각각 독립적으로 제어함으로써 로봇을 이동시키는 방식
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_plugins/gazebo_ros_diff_drive.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sdf/sdf.hh>

#ifdef NO_ERROR
// NO_ERROR is a macro defined in Windows that's used as an enum in tf2
#undef NO_ERROR
#endif

#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{
class GazeboRosDiffDrivePrivate
{
public:
  /// Indicates where the odometry info is coming from
  enum OdomSource
  {
    /// Use an ancoder
    ENCODER = 0,

    /// Use ground truth from simulation world
    WORLD = 1,
  };

  /// Indicates which wheel
  enum
  {
    /// Right wheel
    RIGHT = 0,

    /// Left wheel
    LEFT = 1,
  };

  /// 시뮬레이션 반복마다 호출되는 콜백 함수.
  /// \param[in] _info 업데이트된 시뮬레이션 정보.
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  /// 속도 명령을 수신했을 때 호출되는 콜백 함수.
  /// \param[in] _msg 트위스트 명령 메시지.
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);

  /// 최신 목표 속도에 따라 바퀴 속도를 업데이트하는 함수.
  void UpdateWheelVelocities();

  /// 인코더에 따라 오도메트리를 업데이트하는 함수.
  /// \param[in] _current_time 현재 시뮬레이션 시간.
  void UpdateOdometryEncoder(const gazebo::common::Time & _current_time);

  /// 월드에 따라 오도메트리를 업데이트하는 함수. ( 전달할 메시지 구성)
  void UpdateOdometryWorld();

  /// 오도메트리 변환을 발행하는 함수.
  /// \param[in] _current_time 현재 시뮬레이션 시간.
  void PublishOdometryTf(const gazebo::common::Time & _current_time);

  /// 바퀴에 대한 변환을 발행하는 함수.
  /// \param[in] _current_time 현재 시뮬레이션 시간.
  void PublishWheelsTf(const gazebo::common::Time & _current_time);

  /// 오도메트리 메시지를 발행하는 함수.
  /// \param[in] _current_time 현재 시뮬레이션 시간.
  void PublishOdometryMsg(const gazebo::common::Time & _current_time);

  /// GazeboROS 노드에 대한 포인터.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// 명령 속도를 수신하는 ROS 구독자.
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  /// 오도메트리를 발행하는 ROS 발행자.
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  /// 모든 월드 반복에서 호출되는 이벤트에 대한 연결.
  gazebo::event::ConnectionPtr update_connection_;

  /// 바퀴 간의 거리, 미터 단위.
  std::vector<double> wheel_separation_;

  /// 바퀴의 지름, 미터 단위.
  std::vector<double> wheel_diameter_;

  /// 바퀴의 최대 토크, 뉴턴 미터 단위.
  double max_wheel_torque_;

  /// 바퀴의 최대 가속도.
  double max_wheel_accel_;

  /// 원하는 바퀴 속도.
  std::vector<double> desired_wheel_speed_;

  /// 바퀴로 전송된 속도.
  std::vector<double> wheel_speed_instr_;

  /// 바퀴 관절에 대한 포인터.
  std::vector<gazebo::physics::JointPtr> joints_;

  /// 모델에 대한 포인터.
  gazebo::physics::ModelPtr model_;

  /// TF(Topic Publisher)를 브로드캐스트하기 위한 공유 포인터.
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  /// 콜백에서 접근되는 변수를 보호하는 뮤텍스.
  std::mutex lock_;

  /// 명령으로 받은 X축 선형 속도, 미터/초 단위.
  double target_x_{0.0};

  /// 명령으로 받은 Z축 각속도, 라디안/초 단위.
  double target_rot_{0.0};

  /// 업데이트 주기, 초 단위.
  double update_period_;

  /// 마지막 업데이트 시간.
  gazebo::common::Time last_update_time_;

  /// 인코더 데이터 유지.
  geometry_msgs::msg::Pose2D pose_encoder_;

  /// 오도메트리 프레임 식별자.
  std::string odometry_frame_;

  /// 인코더가 마지막으로 업데이트된 시간.
  gazebo::common::Time last_encoder_update_;

  /// ENCODER 또는 WORLD 중 하나.
  OdomSource odom_source_;

  /// 최신 오도메트리 메시지 유지.
  nav_msgs::msg::Odometry odom_;

  /// 로봇 기본 프레임 식별자.
  std::string robot_base_frame_;

  /// 오도메트리 메시지를 발행할지 여부.
  bool publish_odom_;

  /// 바퀴에서 기본 변환을 발행할지 여부.
  bool publish_wheel_tf_;

  /// 오도메트리에서 월드 변환을 발행할지 여부.
  bool publish_odom_tf_;

  /// 바퀴 쌍의 개수 저장.
  unsigned int num_wheel_pairs_;

  /// 오도메트리의 공분산.
  double covariance_[3];
};
```

GazeboRosDiffDrive::GazeboRosDiffDrive()
: impl_(std::make_unique<GazeboRosDiffDrivePrivate>())
{
}

GazeboRosDiffDrive::~GazeboRosDiffDrive()
{
}


//ROS 기반 차량 제어 플러그인인 GazeboRosDiffDrive 클래스의 Load 함수. Gazebo 모델이 로드될 때 호출되며, 모델의 파라미터 및 초기화를 수행
void GazeboRosDiffDrive::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // Get number of wheel pairs in the model
  impl_->num_wheel_pairs_ = static_cast<unsigned int>(_sdf->Get<int>("num_wheel_pairs", 1).first);

  if (impl_->num_wheel_pairs_ < 1) {
    impl_->num_wheel_pairs_ = 1;
    RCLCPP_WARN(
      impl_->ros_node_->get_logger(),
      "Drive requires at least one pair of wheels. Setting [num_wheel_pairs] to 1");
  }

  // Dynamic properties
  // impl_ 객체의 max_wheel_accel_ 멤버 변수
  //SDF에서 max_wheel_acceleration 파라미터 값을 읽어오는데, 만약 해당 파라미터가 존재하지 않으면 기본값으로 0.0을 사용
  impl_->max_wheel_accel_ = _sdf->Get<double>("max_wheel_acceleration", 0.0).first;
  impl_->max_wheel_torque_ = _sdf->Get<double>("max_wheel_torque", 5.0).first;

  // Get joints and Kinematic properties
  std::vector<gazebo::physics::JointPtr> left_joints, right_joints;



	//SDF에서 "left_joint"라는 요소를 가져옴				// "left_joint" 요소가 존재하는 동안에만 루프를 실행
  for (auto left_joint_elem = _sdf->GetElement("left_joint"); left_joint_elem != nullptr;
    left_joint_elem = left_joint_elem->GetNextElement("left_joint"))// 루프마다 다음 요소 ( i++)
  {
    auto left_joint_name = left_joint_elem->Get<std::string>();
    auto left_joint = _model->GetJoint(left_joint_name);
    if (!left_joint) {
      RCLCPP_ERROR(
        impl_->ros_node_->get_logger(),
        "Joint [%s] not found, plugin will not work.", left_joint_name.c_str());
      impl_->ros_node_.reset();
      return;
    }
    left_joint->SetParam("fmax", 0, impl_->max_wheel_torque_); // 0: fmax 파라미터의 0번째 인덱스( 인자)
    left_joints.push_back(left_joint);
  }



  for (auto right_joint_elem = _sdf->GetElement("right_joint"); right_joint_elem != nullptr;
    right_joint_elem = right_joint_elem->GetNextElement("right_joint"))
  {
    auto right_joint_name = right_joint_elem->Get<std::string>();
    auto right_joint = _model->GetJoint(right_joint_name);
    if (!right_joint) {
      RCLCPP_ERROR(
        impl_->ros_node_->get_logger(),
        "Joint [%s] not found, plugin will not work.", right_joint_name.c_str());
      impl_->ros_node_.reset();
      return;
    }
    right_joint->SetParam("fmax", 0, impl_->max_wheel_torque_);
    right_joints.push_back(right_joint);
  }

  if (left_joints.size() != right_joints.size() || left_joints.size() != impl_->num_wheel_pairs_) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Inconsistent number of joints specified. Plugin will not work.");
    impl_->ros_node_.reset();
    return;
  }




	//좌우 조인트 객체 정보가 번갈아 저장.
  unsigned int index;
  for (index = 0; index < impl_->num_wheel_pairs_; ++index) {
    impl_->joints_.push_back(right_joints[index]);
    impl_->joints_.push_back(left_joints[index]);
  }

  index = 0;
  
  //바퀴간 거리
  // impl_->num_wheel_pairs_로 wheel_separation_의 값을 개수만큼 0.34로 초기화
  impl_->wheel_separation_.assign(impl_->num_wheel_pairs_, 0.34);
  
  for (auto wheel_separation = _sdf->GetElement("wheel_separation"); wheel_separation != nullptr;
    wheel_separation = wheel_separation->GetNextElement("wheel_separation"))
  {
    if (index >= impl_->num_wheel_pairs_) {
      RCLCPP_WARN(impl_->ros_node_->get_logger(), "Ignoring rest of specified <wheel_separation>");
      break;
    }
    impl_->wheel_separation_[index] = wheel_separation->Get<double>();
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(),
      "Wheel pair %i separation set to [%fm]", index + 1, impl_->wheel_separation_[index]);
    index++;
  }


	//바퀴 지름
  index = 0;
  impl_->wheel_diameter_.assign(impl_->num_wheel_pairs_, 0.15);
  for (auto wheel_diameter = _sdf->GetElement("wheel_diameter"); wheel_diameter != nullptr;
    wheel_diameter = wheel_diameter->GetNextElement("wheel_diameter"))
  {
    if (index >= impl_->num_wheel_pairs_) {
      RCLCPP_WARN(impl_->ros_node_->get_logger(), "Ignoring rest of specified <wheel_diameter>");
      break;
    }
    impl_->wheel_diameter_[index] = wheel_diameter->Get<double>();
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(),
      "Wheel pair %i diameter set to [%fm]", index + 1, impl_->wheel_diameter_[index]);
    index++;
  }



  impl_->wheel_speed_instr_.assign(2 * impl_->num_wheel_pairs_, 0);
  impl_->desired_wheel_speed_.assign(2 * impl_->num_wheel_pairs_, 0);


  // Update rate
  auto update_rate = _sdf->Get<double>("update_rate", 100.0).first;
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }
  impl_->last_update_time_ = _model->GetWorld()->SimTime();

	// cmd_vel 구독 생성
  impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
    std::bind(&GazeboRosDiffDrivePrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(), "Subscribed to [%s]",
    impl_->cmd_vel_sub_->get_topic_name());


  // Odometry 프레임, 정보 소스 및 로봇 base 프레임 설정
  // odometry_frame 태그가 있으면 이를 프레임으로 사용 // 없으면 odom이 사용됨
  impl_->odometry_frame_ = _sdf->Get<std::string>("odometry_frame", "odom").first;
  //만약 태그가 존재하지 않으면 기본값으로 "base_footprint" 사용
  impl_->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_footprint").first;
  //태그가 존재하지 않으면 기본값으로 1 사용
  impl_->odom_source_ = static_cast<GazeboRosDiffDrivePrivate::OdomSource>(
    _sdf->Get<int>("odometry_source", 1).first);

  // Advertise odometry topic
  impl_->publish_odom_ = _sdf->Get<bool>("publish_odom", false).first;
  if (impl_->publish_odom_) {
    impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
      "odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Advertise odometry on [%s]",
      impl_->odometry_pub_->get_topic_name());
  }


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

  impl_->covariance_[0] = _sdf->Get<double>("covariance_x", 0.00001).first;
  impl_->covariance_[1] = _sdf->Get<double>("covariance_y", 0.00001).first;
  impl_->covariance_[2] = _sdf->Get<double>("covariance_yaw", 0.001).first;

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosDiffDrivePrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

//초기화
void GazeboRosDiffDrive::Reset()
{
  impl_->last_update_time_ =
    impl_->joints_[GazeboRosDiffDrivePrivate::LEFT]->GetWorld()->SimTime();
  for (unsigned int i = 0; i < impl_->num_wheel_pairs_; ++i) {
    if (impl_->joints_[2 * i + GazeboRosDiffDrivePrivate::LEFT] &&
      impl_->joints_[2 * i + GazeboRosDiffDrivePrivate::RIGHT])
    {
      impl_->joints_[2 * i + GazeboRosDiffDrivePrivate::LEFT]->SetParam(
        "fmax", 0, impl_->max_wheel_torque_); // fmax(최대 토크)를 최대 토크로 설정
      impl_->joints_[2 * i + GazeboRosDiffDrivePrivate::RIGHT]->SetParam(
        "fmax", 0, impl_->max_wheel_torque_);
    }
  }
  impl_->pose_encoder_.x = 0;
  impl_->pose_encoder_.y = 0;
  impl_->pose_encoder_.theta = 0;
  impl_->target_x_ = 0;
  impl_->target_rot_ = 0;
}
	// 업데이트된 시뮬 정보로 내용을 업데이트
void GazeboRosDiffDrivePrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosDiffDrivePrivate::OnUpdate");
#endif
  // Update encoder even if we're going to skip this update
  if (odom_source_ == ENCODER) {
    UpdateOdometryEncoder(_info.simTime);
  }

  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();

  if (seconds_since_last_update < update_period_) {
    return;
  }

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("UpdateOdometryWorld");
#endif
  // Update odom message if using ground truth
  if (odom_source_ == WORLD) {
    UpdateOdometryWorld();
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("PublishOdometryMsg");
#endif
  if (publish_odom_) {
    PublishOdometryMsg(_info.simTime);
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("PublishWheelsTf");
#endif
  if (publish_wheel_tf_) {
    PublishWheelsTf(_info.simTime);
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("PublishOdometryTf");
#endif
  if (publish_odom_tf_) {
    PublishOdometryTf(_info.simTime);
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("UpdateWheelVelocities");
#endif
  // Update robot in case new velocities have been requested
  UpdateWheelVelocities();
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif

	// 메인 부분 : 속도 업데이트, 목표 속도까지 증가 후 유지
  // Current speed
  std::vector<double> current_speed(2 * num_wheel_pairs_);
  for (unsigned int i = 0; i < num_wheel_pairs_; ++i) {
    current_speed[2 * i + LEFT] =
      joints_[2 * i + LEFT]->GetVelocity(0) * (wheel_diameter_[i] / 2.0);
    current_speed[2 * i + RIGHT] =
      joints_[2 * i + RIGHT]->GetVelocity(0) * (wheel_diameter_[i] / 2.0);
  }

  // If max_accel == 0, or target speed is reached
  for (unsigned int i = 0; i < num_wheel_pairs_; ++i) {
    if (max_wheel_accel_ == 0 ||
      ((fabs(desired_wheel_speed_[2 * i + LEFT] - current_speed[2 * i + LEFT]) < 0.01) &&
      (fabs(desired_wheel_speed_[2 * i + RIGHT] - current_speed[2 * i + RIGHT]) < 0.01)))
    {
      joints_[2 * i + LEFT]->SetParam(
        "vel", 0, desired_wheel_speed_[2 * i + LEFT] / (wheel_diameter_[i] / 2.0));
      joints_[2 * i + RIGHT]->SetParam(
        "vel", 0, desired_wheel_speed_[2 * i + RIGHT] / (wheel_diameter_[i] / 2.0));
    } else {
      if (desired_wheel_speed_[2 * i + LEFT] >= current_speed[2 * i + LEFT]) {
        wheel_speed_instr_[2 * i + LEFT] += fmin(
          desired_wheel_speed_[2 * i + LEFT] -
          current_speed[2 * i + LEFT], max_wheel_accel_ * seconds_since_last_update);
      } else {
        wheel_speed_instr_[2 * i + LEFT] += fmax(
          desired_wheel_speed_[2 * i + LEFT] -
          current_speed[2 * i + LEFT], -max_wheel_accel_ * seconds_since_last_update);
      }

      if (desired_wheel_speed_[2 * i + RIGHT] > current_speed[2 * i + RIGHT]) {
        wheel_speed_instr_[2 * i + RIGHT] += fmin(
          desired_wheel_speed_[2 * i + RIGHT] -
          current_speed[2 * i + RIGHT], max_wheel_accel_ * seconds_since_last_update);
      } else {
        wheel_speed_instr_[2 * i + RIGHT] += fmax(
          desired_wheel_speed_[2 * i + RIGHT] -
          current_speed[2 * i + RIGHT], -max_wheel_accel_ * seconds_since_last_update);
      }

      joints_[2 * i + LEFT]->SetParam(
        "vel", 0, wheel_speed_instr_[2 * i + LEFT] / (wheel_diameter_[i] / 2.0));
      joints_[2 * i + RIGHT]->SetParam(
        "vel", 0, wheel_speed_instr_[2 * i + RIGHT] / (wheel_diameter_[i] / 2.0));
    }
  }

  last_update_time_ = _info.simTime;
}

//바퀴 속도 업데이트
void GazeboRosDiffDrivePrivate::UpdateWheelVelocities()
{
  std::lock_guard<std::mutex> scoped_lock(lock_);

  double vr = target_x_;	//선형속도
  double va = target_rot_;	//각속도

  for (unsigned int i = 0; i < num_wheel_pairs_; ++i) {
    desired_wheel_speed_[2 * i + LEFT] = vr - va * wheel_separation_[i] / 2.0;
    desired_wheel_speed_[2 * i + RIGHT] = vr + va * wheel_separation_[i] / 2.0;
  }
}

//업데이트된 속도값으로 업데이트
void GazeboRosDiffDrivePrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  target_x_ = _msg->linear.x;
  target_rot_ = _msg->angular.z;
}

//odom을 업데이트
void GazeboRosDiffDrivePrivate::UpdateOdometryEncoder(const gazebo::common::Time & _current_time)
{
  double vl = joints_[LEFT]->GetVelocity(0);	//	left
  double vr = joints_[RIGHT]->GetVelocity(0);	//	right

  double seconds_since_last_update = (_current_time - last_encoder_update_).Double();
  last_encoder_update_ = _current_time;

  double b = wheel_separation_[0]; //바퀴 쌍 간의 거리



  // Book: Sigwart 2011 Autonompus Mobile Robots page:337
  // 이동 거리를 계산하여 로봇의 운동 상태를 추정
  
  double sl = vl * (wheel_diameter_[0] / 2.0) * seconds_since_last_update;
  double sr = vr * (wheel_diameter_[0] / 2.0) * seconds_since_last_update;
  double ssum = sl + sr; // 직선 거리

  double sdiff = sr - sl; // 회전 거리

	// 이동거리 , 회전반경
  double dx = (ssum) / 2.0 * cos(pose_encoder_.theta + (sdiff) / (2.0 * b));
  double dy = (ssum) / 2.0 * sin(pose_encoder_.theta + (sdiff) / (2.0 * b));
  double dtheta = (sdiff) / b;

	//업데이트
  pose_encoder_.x += dx;
  pose_encoder_.y += dy;
  pose_encoder_.theta += dtheta;

	//각속도(w),선속도(v)
  double w = dtheta / seconds_since_last_update;
  double v = sqrt(dx * dx + dy * dy) / seconds_since_last_update;

  tf2::Quaternion qt;
  tf2::Vector3 vt;
  qt.setRPY(0, 0, pose_encoder_.theta);
  vt = tf2::Vector3(pose_encoder_.x, pose_encoder_.y, 0);

	// odom 객체 업데이트
  odom_.pose.pose.position.x = vt.x();
  odom_.pose.pose.position.y = vt.y();
  odom_.pose.pose.position.z = vt.z();

  odom_.pose.pose.orientation.x = qt.x();
  odom_.pose.pose.orientation.y = qt.y();
  odom_.pose.pose.orientation.z = qt.z();
  odom_.pose.pose.orientation.w = qt.w();

  odom_.twist.twist.angular.z = w;
  odom_.twist.twist.linear.x = v;
  odom_.twist.twist.linear.y = 0;
}


//가제보에서 로봇의 위치와 속도를 구현 (메시지에 저장하는 역할)
void GazeboRosDiffDrivePrivate::UpdateOdometryWorld()
{
  auto pose = model_->WorldPose();
  odom_.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
  odom_.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

  // Get velocity in odom frame 
  auto linear = model_->WorldLinearVel();
  odom_.twist.twist.angular.z = model_->WorldAngularVel().Z();

  // Convert velocity to child_frame_id(aka base_footprint) 축 방향의 선속도를 계산
  float yaw = pose.Rot().Yaw();
  odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
  odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
}

//위에서 정한 odom을 TF(Transform) 메시지로 변환하여 브로드캐스트하는 역할
void GazeboRosDiffDrivePrivate::PublishOdometryTf(const gazebo::common::Time & _current_time)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
  msg.header.frame_id = odometry_frame_;
  msg.child_frame_id = robot_base_frame_;
  msg.transform.translation =
    gazebo_ros::Convert<geometry_msgs::msg::Vector3>(odom_.pose.pose.position);
  msg.transform.rotation = odom_.pose.pose.orientation;

  transform_broadcaster_->sendTransform(msg);
}

//각 바퀴의 TF(Transform) 메시지를 생성하고 브로드캐스트하는 역할
void GazeboRosDiffDrivePrivate::PublishWheelsTf(const gazebo::common::Time & _current_time)
{
  for (unsigned int i = 0; i < 2 * num_wheel_pairs_; ++i) {
    auto pose_wheel = joints_[i]->GetChild()->RelativePose();

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
    msg.header.frame_id = joints_[i]->GetParent()->GetName();
    msg.child_frame_id = joints_[i]->GetChild()->GetName();
    msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(pose_wheel.Pos());
    msg.transform.rotation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose_wheel.Rot());

    transform_broadcaster_->sendTransform(msg);
  }
}
//로봇의 오도메트리 정보를 ROS 메시지로 변환하고, 지정된 covariance 값으로 메시지 설정하고 ros 노드로 송신
void GazeboRosDiffDrivePrivate::PublishOdometryMsg(const gazebo::common::Time & _current_time)
{
  // Set covariance
  odom_.pose.covariance[0] = covariance_[0];
  odom_.pose.covariance[7] = covariance_[1];
  odom_.pose.covariance[14] = 1000000000000.0;
  odom_.pose.covariance[21] = 1000000000000.0;
  odom_.pose.covariance[28] = 1000000000000.0;
  odom_.pose.covariance[35] = covariance_[2];

  odom_.twist.covariance[0] = covariance_[0];
  odom_.twist.covariance[7] = covariance_[1];
  odom_.twist.covariance[14] = 1000000000000.0;
  odom_.twist.covariance[21] = 1000000000000.0;
  odom_.twist.covariance[28] = 1000000000000.0;
  odom_.twist.covariance[35] = covariance_[2];

  // Set header
  odom_.header.frame_id = odometry_frame_;
  odom_.child_frame_id = robot_base_frame_;
  odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);

  // Publish
  odometry_pub_->publish(odom_);
}
GZ_REGISTER_MODEL_PLUGIN(GazeboRosDiffDrive)
}  // namespace gazebo_plugins
