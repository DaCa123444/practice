// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"  // ROS 2의 C++ 액션 라이브러리

#include "control_msgs/action/follow_joint_trajectory.hpp"  // 제어 메시지





std::shared_ptr<rclcpp::Node> node;

bool common_goal_accepted = false;	//전역변수?

rclcpp_action::ResultCode common_resultcode = rclcpp_action::ResultCode::UNKNOWN;

int common_action_result_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;


//FollowJointTrajectory에 대해

//1. 목표값 수신
void common_goal_response( // 인자는 목표값을 갖는 메시지에 대한 핸들
  rclcpp_action::ClientGoalHandle	
  <control_msgs::action::FollowJointTrajectory>::SharedPtr goal_handle)
{
  RCLCPP_DEBUG(
    node->get_logger(), "common_goal_response time: %f",
    rclcpp::Clock().now().seconds());
  if (!goal_handle) {
    common_goal_accepted = false;
    printf("Goal rejected\n");
  } else {
    common_goal_accepted = true;
    printf("Goal accepted\n");
  }
}

//2. 결과값 수신
void common_result_response(
  const rclcpp_action::ClientGoalHandle
  <control_msgs::action::FollowJointTrajectory>::WrappedResult & result)
{
  printf("common_result_response time: %f\n", rclcpp::Clock().now().seconds());
  common_resultcode = result.code;
  common_action_result_code = result.result->error_code;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      printf("SUCCEEDED result code\n");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      printf("Goal was aborted\n");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      printf("Goal was canceled\n");
      return;
    default:
      printf("Unknown result code\n");
      return;
  }
}

//3. 콜백 원하는 위치와 속도값을 출력
void common_feedback(
  rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
  const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
{
  std::cout << "feedback->desired.positions :";
  for (auto & x : feedback->desired.positions) {
    std::cout << x << "\t";
  }
  std::cout << std::endl;
  std::cout << "feedback->desired.velocities :";
  for (auto & x : feedback->desired.velocities) {
    std::cout << x << "\t";
  }
  std::cout << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("trajectory_test_node");

  std::cout << "node created" << std::endl;

  // 액션 클라이언트 생성
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client;
  action_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
    node->get_node_base_interface(),
    node->get_node_graph_interface(),
    node->get_node_logging_interface(),
    node->get_node_waitables_interface(),
    "/joint_trajectory_controller/follow_joint_trajectory");

  // 액션 서버 응답 대기
  bool response =
    action_client->wait_for_action_server(std::chrono::seconds(1));
  if (!response) {
    throw std::runtime_error("could not get action server");
  }
  std::cout << "Created action server" << std::endl;
  
  // 조인트 이름 설정
  std::vector<std::string> joint_names = {"slider_to_cart"};

  // 조인트 트라젝토리 포인트 설정
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;

  // 첫 번째 포인트 설정
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(0.0);  // start asap
  point.positions.resize(joint_names.size());
  point.positions[0] = 0.0;
  points.push_back(point);
  
  trajectory_msgs::msg::JointTrajectoryPoint point2;
  point2.time_from_start = rclcpp::Duration::from_seconds(1.0);
  point2.positions.resize(joint_names.size());
  point2.positions[0] = -1.0;
  points.push_back(point2);
  
  trajectory_msgs::msg::JointTrajectoryPoint point3;
  point3.time_from_start = rclcpp::Duration::from_seconds(2.0);
  point3.positions.resize(joint_names.size());
  point3.positions[0] = 1.0;
  points.push_back(point3);
  
  trajectory_msgs::msg::JointTrajectoryPoint point4;
  point4.time_from_start = rclcpp::Duration::from_seconds(3.0);
  point4.positions.resize(joint_names.size());
  point4.positions[0] = 0.0;
  points.push_back(point4);




  // 클라 목표 옵션 / 목표 응답 // 결과 / 피드백 설정
  // common_이 붙은 것들이 위에서 생성된 함수들
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions opt;
  opt.goal_response_callback = std::bind(common_goal_response, std::placeholders::_1);
  opt.result_callback = std::bind(common_result_response, std::placeholders::_1);
  opt.feedback_callback = std::bind(common_feedback, std::placeholders::_1, std::placeholders::_2);

  // FollowJointTrajectory 액션의 목표 메시지 생성 및 설정
  control_msgs::action::FollowJointTrajectory_Goal goal_msg;
  goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(1.0);
  goal_msg.trajectory.joint_names = joint_names;
  goal_msg.trajectory.points = points;
  
  // 액션 클라이언트를 통해 목표를 비동기적(async)으로 전송하고, 그 결과를 받아오는 Future 객체 생성
  auto goal_handle_future = action_client->async_send_goal(goal_msg, opt);

  // 전송이 성공적으로 완료될 때까지 루프 돌리기
  if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "send goal call failed :(");
    action_client.reset();
    node.reset();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "send goal call ok :)");
  
  // 액션 목표 핸들 획득
  rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr
    goal_handle = goal_handle_future.get();
    
  // 액션 목표 핸들이 없으면 에러 출력 및 종료
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
    action_client.reset();
    node.reset();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Goal was accepted by server");

  // Wait for the server to be done with the goal
  auto result_future = action_client->async_get_result(goal_handle);
  RCLCPP_INFO(node->get_logger(), "Waiting for result");
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "get result call failed :(");
    return 1;
  }
  // 메모리 정리 및 ROS 2 종료
  action_client.reset();
  node.reset();
  rclcpp::shutdown();

  return 0;
}
