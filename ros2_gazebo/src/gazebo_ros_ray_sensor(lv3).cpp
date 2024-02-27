// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <boost/make_shared.hpp>
#include <boost/variant.hpp>
#include <gazebo/transport/transport.hh>
#include <gazebo_plugins/gazebo_ros_ray_sensor.hpp>
#include <gazebo_ros/conversions/sensor_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <algorithm>
#include <limits>
#include <memory>

namespace gazebo_plugins
{

class GazeboRosRaySensorPrivate
{
public:
	//노드 포인터
  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

	// 메시지 타입/포인터로 퍼블리셔 객체
  // Aliases
  using LaserScan = sensor_msgs::msg::LaserScan;
  using PointCloud = sensor_msgs::msg::PointCloud;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Range = sensor_msgs::msg::Range;
  using LaserScanPub = rclcpp::Publisher<LaserScan>::SharedPtr;
  using PointCloudPub = rclcpp::Publisher<PointCloud>::SharedPtr;
  using PointCloud2Pub = rclcpp::Publisher<PointCloud2>::SharedPtr;
  using RangePub = rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr;

  /// Publisher of output
  /// \todo use std::variant one c++17 is supported in ROS2
  //boost::variant를 사용하여 여러 타입의 ROS 퍼블리셔(Publisher) 중 하나를 저장할 수 있는 pub_ 변수를 선언
  boost::variant<LaserScanPub, PointCloudPub, PointCloud2Pub, RangePub> pub_;

  /// TF frame output 
  std::string frame_name_;

  /// Subscribe to gazebo's laserscan, calling the appropriate callback based on output type
  //Gazebo의 레이저 스캔을 구독하는 메소드. 출력 유형에 따라 적절한 콜백을 호출
  void SubscribeGazeboLaserScan();

  /// Publish a sensor_msgs/LaserScan message from a gazebo laser scan
  //Gazebo 레이저 스캔에서 sensor_msgs/LaserScan 메시지를 발행
  void PublishLaserScan(ConstLaserScanStampedPtr & _msg);

  /// Publish a sensor_msgs/PointCloud message from a gazebo laser scan
  //sensor_msgs/PointCloud 메시지를 발행
  void PublishPointCloud(ConstLaserScanStampedPtr & _msg);

  /// Publish a sensor_msgs/PointCloud2 message from a gazebo laser scan
  //PointCloud2 메시지를 발행
  void PublishPointCloud2(ConstLaserScanStampedPtr & _msg);

  /// Publish a sensor_msgs/Range message from a gazebo laser scan
  //Range 메시지를 발행
  void PublishRange(ConstLaserScanStampedPtr & _msg);

  /// Gazebo transport topic to subscribe to for laser scan
  //레이저 스캔을 위해 Gazebo에서 구독할 토픽
  std::string sensor_topic_;
  
//레이저 스캔/포인트클라우드 메시지를 발행할 때의 최소 강도 값
  /// Minimum intensity value to publish for laser scan / pointcloud messages
  double min_intensity_{0.0};
  
	//레이저 스캔이 Range 출력 유형일 때 보고할 방사 형태
  /// brief Radiation type to report when output type is range
  uint8_t range_radiation_type_;
  
	//Gazebo에서 레이저 스캔을 구독하기 위해 사용되는 노드
  /// Gazebo node used to subscribe to laser scan
  gazebo::transport::NodePtr gazebo_node_;

  /// Gazebo subscribe to parent sensor's laser scan
  //Gazebo에서 부모 센서의 레이저 스캔을 구독하기 위한 구독자
  gazebo::transport::SubscriberPtr laser_scan_sub_;
};

//생성자
GazeboRosRaySensor::GazeboRosRaySensor()
: impl_(std::make_unique<GazeboRosRaySensorPrivate>())
{
}
//소멸자
GazeboRosRaySensor::~GazeboRosRaySensor()
{
  // Must release subscriber and then call fini on node to remove it from topic manager.
  //laser_scan_sub_을 리셋하여 구독자를 해제하고, gazebo_node_가 존재하는 경우 Fini()를 호출하여 노드를 종료하고 gazebo_node_을 리셋
  impl_->laser_scan_sub_.reset();
  if (impl_->gazebo_node_) {
    impl_->gazebo_node_->Fini();
  }
  impl_->gazebo_node_.reset();
}

void GazeboRosRaySensor::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{

//sdf파일로부터 xml 형식을 생성하고 / 통신 설정(qos) / sdf에서 제공된 정보에 따라 적절한 pub을 생성

  // Create ros_node configured from sdf
  //ROS 노드를 SDF(ROS에서 사용하는 파라미터 서버의 XML 형식)로부터 설정하여 생성
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles// ROS 노드의 QoS(Quality of Service) 설정을 가짐
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // Get QoS profile for the publisher// Publisher를 위한 QoS 프로파일을 가져옴 "~/out" 토픽에 대한 신뢰성 있는(QoS 설정) Publisher를 생성
  
  rclcpp::QoS pub_qos = qos.get_publisher_qos("~/out", rclcpp::SensorDataQoS().reliable());

  // Get tf frame for output// 레이저 센서의 tf 프레임 이름을 가짐
  impl_->frame_name_ = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  // Get output type from sdf if provided// SDF에서 제공된 경우 출력 유형(output type)을 가짐 
  if (!_sdf->HasElement("output_type")) {
    RCLCPP_WARN(// "output_type"이 없는 경우 기본값으로 "sensor_msgs/PointCloud2"를 사용하여 Publisher를 생성
    
      impl_->ros_node_->get_logger(), "missing <output_type>, defaults to sensor_msgs/PointCloud2");
    impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/out", pub_qos);
  } else {
    std::string output_type_string = _sdf->Get<std::string>("output_type");
    // "output_type"이 제공된 경우 해당 유형에 따라 적절한 Publisher를 생성
    if (output_type_string == "sensor_msgs/LaserScan") {
      impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::LaserScan>(
        "~/out", pub_qos);
    } else if (output_type_string == "sensor_msgs/PointCloud") {
      impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::PointCloud>(
        "~/out", pub_qos);
    } else if (output_type_string == "sensor_msgs/PointCloud2") {
      impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/out", pub_qos);
    } else if (output_type_string == "sensor_msgs/Range") {
      impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::Range>(
        "~/out", pub_qos);
    } else {
      RCLCPP_ERROR(// 유효하지 않은 "output_type"이 제공된 경우 오류를 출력하고 종료
        impl_->ros_node_->get_logger(), "Invalid <output_type> [%s]", output_type_string.c_str());
      return;
    }
  }
// Range 출력 유형에 특화된 매개변수를 SDF로부터 가져옴
  // Get parameters specific to Range output from sdf
  if (impl_->pub_.type() == typeid(GazeboRosRaySensorPrivate::RangePub)) {
  // "radiation_type" 파라미터가 설정되지 않은 경우 기본값으로 "infrared(적외선)"를 사용
  
    if (!_sdf->HasElement("radiation_type")) {
      RCLCPP_INFO(
        impl_->ros_node_->get_logger(), "missing <radiation_type>, defaulting to infrared");
      impl_->range_radiation_type_ = sensor_msgs::msg::Range::INFRARED;
    } else if ("ultrasound" == _sdf->Get<std::string>("radiation_type")) {
      impl_->range_radiation_type_ = sensor_msgs::msg::Range::ULTRASOUND;
    } else if ("infrared" == _sdf->Get<std::string>("radiation_type")) {
      impl_->range_radiation_type_ = sensor_msgs::msg::Range::INFRARED;
    } else {
      RCLCPP_ERROR(// 유효하지 않은 "radiation_type"이 설정된 경우 오류를 출력하고 종료
        impl_->ros_node_->get_logger(),
        "Invalid <radiation_type> [%s]. Can be ultrasound or infrared",
        _sdf->Get<std::string>("radiation_type").c_str());
      return;
    }
  }
  
// "min_intensity" 파라미터가 설정되지 않은 경우 디버그 메시지를 출력하고 기본값을 설정
  if (!_sdf->HasElement("min_intensity")) {
    RCLCPP_DEBUG(
      impl_->ros_node_->get_logger(), "missing <min_intensity>, defaults to %f",
      impl_->min_intensity_);
  } else {
  // "min_intensity" 파라미터가 설정된 경우 해당 값을 가져와 설정
    impl_->min_intensity_ = _sdf->Get<double>("min_intensity");
  }

// Gazebo 전송 노드를 생성하고 센서의 레이저 스캔에 구독
//Gazebo 전송 노드를 생성하고 센서의 레이저 스캔 데이터에 구독하는 부분
  // Create gazebo transport node and subscribe to sensor's laser scan
  impl_->gazebo_node_ = boost::make_shared<gazebo::transport::Node>();
  impl_->gazebo_node_->Init(_sensor->WorldName());
  
// lazy publisher를 사용하여 출력이 구독자가 있는 경우에만 레이저 데이터를 처리
  // TODO(ironmig): use lazy publisher to only process laser data when output has a subscriber
  impl_->sensor_topic_ = _sensor->Topic(); //Topic() : 가제보 센서 토픽 이름
  impl_->SubscribeGazeboLaserScan(); //SubscribeGazeboLaserScan 함수가 호출되어 해당 토픽에서 발생하는 레이저 스캔 데이터에 대한 구독을 설정
}


// 레이저 스캔을 구독하고 해당 데이터를 ROS 메시지로 변환하여 발행(Publish)하는 함수
  // 현재 Publisher의 타입에 따라 적절한 콜백 함수를 등록
void GazeboRosRaySensorPrivate::SubscribeGazeboLaserScan()
{
  if (pub_.type() == typeid(LaserScanPub)) {
    laser_scan_sub_ = gazebo_node_->Subscribe(
      sensor_topic_, &GazeboRosRaySensorPrivate::PublishLaserScan, this);
  } else if (pub_.type() == typeid(PointCloudPub)) {
    laser_scan_sub_ = gazebo_node_->Subscribe(
      sensor_topic_, &GazeboRosRaySensorPrivate::PublishPointCloud, this);
  } else if (pub_.type() == typeid(PointCloud2Pub)) {
    laser_scan_sub_ = gazebo_node_->Subscribe(
      sensor_topic_, &GazeboRosRaySensorPrivate::PublishPointCloud2, this);
  } else if (pub_.type() == typeid(RangePub)) {
    laser_scan_sub_ = gazebo_node_->Subscribe(
      sensor_topic_, &GazeboRosRaySensorPrivate::PublishRange, this);
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "Publisher is an invalid type. This is an internal bug.");
  }
}
// LaserScan을 ROS LaserScan으로 변환하여 출력하는 함수
void GazeboRosRaySensorPrivate::PublishLaserScan(ConstLaserScanStampedPtr & _msg)
{
  // Convert Laser scan to ROS LaserScan// Gazebo LaserScan을 ROS LaserScan으로 변환
  auto ls = gazebo_ros::Convert<sensor_msgs::msg::LaserScan>(*_msg);
  // Set tf frame
  ls.header.frame_id = frame_name_;
  // Publish output
  boost::get<LaserScanPub>(pub_)->publish(ls);
}

// LaserScan을 PointCloud로 변환하여 출력하는 함수
void GazeboRosRaySensorPrivate::PublishPointCloud(ConstLaserScanStampedPtr & _msg)
{
  // Convert Laser scan to PointCloud
  auto pc = gazebo_ros::Convert<sensor_msgs::msg::PointCloud>(*_msg, min_intensity_);
  // Set tf frame
  pc.header.frame_id = frame_name_;
  // Publish output
  boost::get<PointCloudPub>(pub_)->publish(pc);
}

// LaserScan을 PointCloud2로 변환하여 출력하는 함수
void GazeboRosRaySensorPrivate::PublishPointCloud2(ConstLaserScanStampedPtr & _msg)
{
  // Convert Laser scan to PointCloud2// Gazebo LaserScan을 PointCloud로 변환
  auto pc2 = gazebo_ros::Convert<sensor_msgs::msg::PointCloud2>(*_msg, min_intensity_);
  // Set tf frame
  pc2.header.frame_id = frame_name_;
  // Publish output
  boost::get<PointCloud2Pub>(pub_)->publish(pc2);
}

// LaserScan을 range로 변환하여 출력하는 함수
void GazeboRosRaySensorPrivate::PublishRange(ConstLaserScanStampedPtr & _msg)
{
  // Convert Laser scan to range
  auto range_msg = gazebo_ros::Convert<sensor_msgs::msg::Range>(*_msg);
  // Set tf frame
  range_msg.header.frame_id = frame_name_;
  // Set radiation type from sdf
  range_msg.radiation_type = range_radiation_type_;
  // Publish output
  boost::get<RangePub>(pub_)->publish(range_msg);
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosRaySensor)

}  // namespace gazebo_plugins
