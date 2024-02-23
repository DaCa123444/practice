#!/usr/bin/env python
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Will Son

# ROS 2와 관련된 모듈 및 메시지, 서비스를 가져옴
from concurrent.futures import ThreadPoolExecutor  # 비동기 작업을 처리하기 위한 스레드 풀 모듈
from math import exp  # 지수 함수를 계산하기 위한 math 모듈의 함수
import os  # 운영 체제와 상호 작용을 위한 모듈
import select  # 입출력 객체의 상태를 감시하기 위한 모듈
import sys  # 파이썬 인터프리터와 관련된 정보를 제공하는 모듈
import rclpy  # ROS 2의 파이썬 클라이언트 라이브러리

from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState  # OpenManipulator 메시지 타입
from open_manipulator_msgs.srv import SetJointPosition, SetKinematicsPose  # OpenManipulator 서비스 타입
from rclpy.callback_groups import ReentrantCallbackGroup  # 콜백 그룹 모듈
from sensor_msgs.msg import JointState  # JointState 메시지 타입
from rclpy.node import Node  # ROS2 노드 클래스
from rclpy.qos import QoSProfile  # ROS 통신 Quality of Service 정의 모듈
from threading import Timer 


# 정의 #
# 이전 목표위치, 현 목표위치, 현 조인트 각도를 배열로 갖음.
# 클래스에서 js,kine_pose,robot_state를 구독하도록 함.
# 클래스에서 여러 서비스 클라이언트를 생성 및 반환함. ( joint space , task space)
# 위 서비스에서 받은 값을 task,joint 객체로 하나로 통합 ( msg -> 객체)

# 기능 #
# 키가 전달되면 메인에서 판단하여 객체를 업데이트, 업데이트 된 객체를 송출


if os.name == 'nt':	#wondow
    import msvcrt
else:
    import termios
    import tty

#여러 종류의 조인트 값
present_joint_angle = [0.0, 0.0, 0.0, 0.0]
goal_joint_angle = [0.0, 0.0, 0.0, 0.0]
prev_goal_joint_angle = [0.0, 0.0, 0.0, 0.0]

#  말단 위치 및 방위
present_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
prev_goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

debug = True
task_position_delta = 0.01  # meter
joint_angle_delta = 0.05  # radian
path_time = 0.5  # second

usage = """
Control Your OpenManipulator!
---------------------------
Task Space Control:
         (Forward, X+)
              W                   Q (Upward, Z+)
(Left, Y+) A     D (Right, Y-)    Z (Downward, Z-)
              X 
        (Backward, X-)

Joint Space Control:
- Joint1 : Increase (Y), Decrease (H)
- Joint2 : Increase (U), Decrease (J)
- Joint3 : Increase (I), Decrease (K)
- Joint4 : Increase (O), Decrease (L)
- Gripper: Open     (G),    Close (F)

INIT : (1)
HOME : (2)

CTRL-C to quit
"""

e = """
Communications Failed
"""


class TeleopKeyboard(Node):

    qos = QoSProfile(depth=10)
    settings = None
    
    if os.name != 'nt':	#windows가 아닐경우
        settings = termios.tcgetattr(sys.stdin) #termios 모듈을 사용하여 터미널 속성을 가져오는 코드

    def __init__(self):
    	
    	#초기화
        super().__init__('teleop_keyboard')
        key_value = '' 

        # Create joint_states subscriber	js sub
        self.joint_state_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            self.qos)
        self.joint_state_subscription

        # Create kinematics_pose subscriber		kine_pos sub
        self.kinematics_pose_subscription = self.create_subscription(
            KinematicsPose,
            'kinematics_pose',
            self.kinematics_pose_callback,
            self.qos)
        self.kinematics_pose_subscription

        # Create manipulator state subscriber	bot_state sub
        self.open_manipulator_state_subscription = self.create_subscription(
            OpenManipulatorState,
            'states',
            self.open_manipulator_state_callback,
            self.qos)
        self.open_manipulator_state_subscription

        # Create Service Clients				srv client
        self.goal_joint_space = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.goal_task_space = self.create_client(SetKinematicsPose, 'goal_task_space_path')
        self.goal_joint_space_req = SetJointPosition.Request()
        self.goal_task_space_req = SetKinematicsPose.Request()

    def send_goal_task_space(self):				# goal ee pose&orientation
        self.goal_task_space_req.end_effector_name = 'gripper'
        self.goal_task_space_req.kinematics_pose.pose.position.x = goal_kinematics_pose[0]
        self.goal_task_space_req.kinematics_pose.pose.position.y = goal_kinematics_pose[1]
        self.goal_task_space_req.kinematics_pose.pose.position.z = goal_kinematics_pose[2]
        self.goal_task_space_req.kinematics_pose.pose.orientation.w = goal_kinematics_pose[3]
        self.goal_task_space_req.kinematics_pose.pose.orientation.x = goal_kinematics_pose[4]
        self.goal_task_space_req.kinematics_pose.pose.orientation.y = goal_kinematics_pose[5]
        self.goal_task_space_req.kinematics_pose.pose.orientation.z = goal_kinematics_pose[6]
        self.goal_task_space_req.path_time = path_time

        try:
            send_goal_task = self.goal_task_space.call_async(self.goal_task_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Kinematic Pose failed %r' % (e,))

    def send_goal_joint_space(self):			# goal js
        self.goal_joint_space_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        self.goal_joint_space_req.joint_position.position = [goal_joint_angle[0], goal_joint_angle[1], goal_joint_angle[2], goal_joint_angle[3]]
        self.goal_joint_space_req.path_time = path_time

        try:		# 데이터 송출
            send_goal_joint = self.goal_joint_space.call_async(self.goal_joint_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Joint failed %r' % (e,))

    def kinematics_pose_callback(self, msg):	#update new data
        present_kinematics_pose[0] = msg.pose.position.x
        present_kinematics_pose[1] = msg.pose.position.y
        present_kinematics_pose[2] = msg.pose.position.z
        present_kinematics_pose[3] = msg.pose.orientation.w
        present_kinematics_pose[4] = msg.pose.orientation.x
        present_kinematics_pose[5] = msg.pose.orientation.y
        present_kinematics_pose[6] = msg.pose.orientation.z

    def joint_state_callback(self, msg):
        present_joint_angle[0] = msg.position[0]
        present_joint_angle[1] = msg.position[1]
        present_joint_angle[2] = msg.position[2]
        present_joint_angle[3] = msg.position[3]

    def open_manipulator_state_callback(self, msg):
        if msg.open_manipulator_moving_state == 'STOPPED':
            for index in range(0, 7):
                goal_kinematics_pose[index] = present_kinematics_pose[index]
            for index in range(0, 4):
                goal_joint_angle[index] = present_joint_angle[index]

def get_key(settings):		# 입력받은 키값 처리하는 방법
    if os.name == 'nt':		#windows
        return msvcrt.getch().decode('utf-8')
    
    #windows가 아니면
    tty.setraw(sys.stdin.fileno())	#터미널 in을 raw로 설정
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)		# 1byte씩 읽음
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)	#터미널 설정 초기화
    print_present_values()			# 출력
    return key

def print_present_values():
    print(usage)
    print('Joint Angle(Rad): [{:.6f}, {:.6f}, {:.6f}, {:.6f}]'.format(
        present_joint_angle[0],#조인트 4개
        present_joint_angle[1],
        present_joint_angle[2],
        present_joint_angle[3]))
    print('Kinematics Pose(Pose X, Y, Z | Orientation W, X, Y, Z): {:.3f}, {:.3f}, {:.3f} | {:.3f}, {:.3f}, {:.3f}, {:.3f}'.format(
        present_kinematics_pose[0],#위치,방위
        present_kinematics_pose[1],
        present_kinematics_pose[2],
        present_kinematics_pose[3],
        present_kinematics_pose[4],
        present_kinematics_pose[5],
        present_kinematics_pose[6]))


def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    try:
        rclpy.init()
    except Exception as e:
        print(e)

    try:
        teleop_keyboard = TeleopKeyboard()
    except Exception as e:
        print(e)

    try:
        while(rclpy.ok()):
            rclpy.spin_once(teleop_keyboard)
            key_value = get_key(settings)
            
            #키 눌리면 목표 위치 변경
            if key_value == 'w':
                goal_kinematics_pose[0] = prev_goal_kinematics_pose[0] + task_position_delta
                teleop_keyboard.send_goal_task_space()
            elif key_value == 'x':
                goal_kinematics_pose[0] = prev_goal_kinematics_pose[0] - task_position_delta
                teleop_keyboard.send_goal_task_space()
            elif key_value == 'a':
                goal_kinematics_pose[1] = prev_goal_kinematics_pose[1] + task_position_delta
                teleop_keyboard.send_goal_task_space()
            elif key_value == 'd':
                goal_kinematics_pose[1] = prev_goal_kinematics_pose[1] - task_position_delta
                teleop_keyboard.send_goal_task_space()
            elif key_value == 'q':
                goal_kinematics_pose[2] = prev_goal_kinematics_pose[2] + task_position_delta
                teleop_keyboard.send_goal_task_space()
            elif key_value == 'z':
                goal_kinematics_pose[2] = prev_goal_kinematics_pose[2] - task_position_delta
                teleop_keyboard.send_goal_task_space()
            elif key_value == 'y':
                goal_joint_angle[0] = prev_goal_joint_angle[0] + joint_angle_delta
                teleop_keyboard.send_goal_joint_space()
            elif key_value == 'h':
                goal_joint_angle[0] = prev_goal_joint_angle[0] - joint_angle_delta
                teleop_keyboard.send_goal_joint_space()
            elif key_value == 'u':
                goal_joint_angle[1] = prev_goal_joint_angle[1] + joint_angle_delta
                teleop_keyboard.send_goal_joint_space()
            elif key_value == 'j':
                goal_joint_angle[1] = prev_goal_joint_angle[1] - joint_angle_delta
                teleop_keyboard.send_goal_joint_space()
            elif key_value == 'i':
                goal_joint_angle[2] = prev_goal_joint_angle[2] + joint_angle_delta
                teleop_keyboard.send_goal_joint_space()
            elif key_value == 'k':
                goal_joint_angle[2] = prev_goal_joint_angle[2] - joint_angle_delta
                teleop_keyboard.send_goal_joint_space()
            elif key_value == 'o':
                goal_joint_angle[3] = prev_goal_joint_angle[3] + joint_angle_delta
                teleop_keyboard.send_goal_joint_space()
            elif key_value == 'l':
                goal_joint_angle[3] = prev_goal_joint_angle[3] - joint_angle_delta
                teleop_keyboard.send_goal_joint_space()
            elif key_value == '1':
                goal_joint_angle[0] = 0.0
                goal_joint_angle[1] = 0.0
                goal_joint_angle[2] = 0.0
                goal_joint_angle[3] = 0.0
                teleop_keyboard.send_goal_joint_space()
            elif key_value == '2':
                goal_joint_angle[0] = 0.0
                goal_joint_angle[1] = -1.05
                goal_joint_angle[2] = 0.35
                goal_joint_angle[3] = 0.70
                teleop_keyboard.send_goal_joint_space()
            else:
                if key_value == '\x03':
                    break
                else:	#해당 키가 없으면 업데이트 x
                    for index in range(0, 7):
                        prev_goal_kinematics_pose[index] = goal_kinematics_pose[index]
                    for index in range(0, 4):
                        prev_goal_joint_angle[index] = goal_joint_angle[index]

    except Exception as e:
        print(e)

    finally:
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        teleop_keyboard.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
