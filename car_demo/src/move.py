#!/usr/bin/env python

# Copyright 2017 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#
# This is a rewrite of the script 'joystick_translator'


import rospy
from rcv_msgs.msg import Control
from rosgraph_msgs.msg import Log
from nav_msgs.msg import Odometry
from PID import PID 



STEERING_AXIS = 0
THROTTLE_AXIS = 4
thre = 0.2


class Translator:
	def __init__(self, operations):
		self.pub = rospy.Publisher('rcv_control_cmd', Control, queue_size=1)
		self.last_published_time = rospy.get_rostime()
		self.operations = operations
		self.timer = rospy.Timer(rospy.Duration(1./20.), self.timer_callback)
		self.linear_v = 0

	def callback(self, data):
		self.linear_v = data.msg
	
	def statecallback(self, data):
		self.x = data.pose.pose.position.x
		self.y = data.pose.pose.position.y

	def timer_callback(self, event):
		if self.last_published_time < rospy.get_rostime() + rospy.Duration(1.0/20.):
			self.move()

	def move(self):
		command = Control()
		self.sub = rospy.Subscriber('rosout_agg', Log, self.callback)
		rospy.Subscriber('/base_pose_ground_truth', Odometry, self.statecallback)
		
		rcv_v = float(self.linear_v)
                ref_v = float(self.operations[1])	
		if self.operations[0] == 1:
                        if  abs(rcv_v - ref_v)/ref_v < thre:
                               command.steer = self.operations[2]
                        else:
                               ref_v = self.operations[1]
			       pid = PID(P=3, I=0.5, D=0)
			       pid.SetPoint = float(ref_v)
		 	       pid.setSampleTime = rospy.Duration(1.0/20.)
			       pid.update(float(rcv_v))

			       if pid.output < 0:
				    command.throttle = 0
				    command.brake = abs(pid.output)
			       else:
				    command.throttle = abs(pid.output)
				    command.brake = 0
		else:
			command.throttle = self.operations[1]
			command.brake = self.operations[2]
			command.shift_gears = self.operations[3]
			command.steer = self.operations[4]

		print("linear velocity: {}, throttle: {}, brake: {}".format(rcv_v, command.throttle, command.brake))
		self.pub.publish(command)


if __name__ == '__main__':
	rospy.init_node('rcv_controller', anonymous=True)

	print("Let's control your RCV")
	m_a = input("Choose automatic (1) or manual (0) control: ")

	if m_a:
		vel = input("Input your velocity: ")
                steer = input("Input your steer: ")
		operations = [m_a, vel, steer]
	else:
		throttle = input("Input your throttle: ")
		brake = input("Input your brake: ")
		shift_gears = input("Input your shift_gears: ")
		steer = input("Input your steer: ")
		operations = [m_a, throttle, brake, shift_gears, steer]

	t = Translator(operations)
	rospy.spin()
