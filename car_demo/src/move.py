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


import os
import rospy
from rcv_msgs.msg import Control
from rcv_msgs.msg import control_command
from rosgraph_msgs.msg import Log
from nav_msgs.msg import Odometry
from PID import PID 
import math
import numpy as np
import liveplot as lp
from matplotlib import pyplot as plt


STEERING_AXIS = 0
THROTTLE_AXIS = 4
thre = 0.2


class Translator:
	def __init__(self, operations=None, planPath=None, scale=None):
		self.operations = operations
		self.planPath = planPath
		self.linear_v = 0
		self.index = 0
		self.x = 3
		self.y = -12
		self.yaw = 0
		self.dist = 0
		self.pre_dist = 0
		self.ref_v = 1.2
		self.scale = scale
		#self.pub = rospy.Publisher('rcv_control_cmd', Control, queue_size=1)
		self.pub = rospy.Publisher('rcv_control_cmd', control_command, queue_size=1)
		self.last_published_time = rospy.get_rostime()
		self.timer = rospy.Timer(rospy.Duration(1.0/20.0), self.timer_callback)
	
	def velcallback(self, data):
		self.linear_v = data.msg
	
	def statecallback(self, data):
		self.x = data.pose.pose.position.x
		self.y = data.pose.pose.position.y
		self.yaw = data.pose.pose.orientation.z
				
	def timer_callback(self, event):
		if self.last_published_time < rospy.get_rostime() + rospy.Duration(1.0/20):
			if self.operations is None:
				self.pathControl()
			else:
				self.move_new()

	# interface takes throttle, brake, steer as input
	def move(self):
		command = Control()
		self.sub = rospy.Subscriber('rosout_agg', Log, self.velcallback)
		rospy.Subscriber('/base_pose_ground_truth', Odometry, self.statecallback)
		
		rcv_v = float(self.linear_v)
		ref_v = float(self.operations[1])	
		
		if self.operations[0]:
			if abs(rcv_v - ref_v)/ref_v < thre:
				command.steer = self.operations[2]
			else:
				ref_v = self.operations[1]
				pid = PID(P=3, I=0.5, D=0)
				pid.SetPoint = float(ref_v)
				pid.setSampleTime = rospy.Duration(1.0/20)
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

	# interface takes torques, kappa, beta as input
	def move_new(self):
		command = control_command()
		self.sub = rospy.Subscriber('rosout_agg', Log, self.velcallback)
		rospy.Subscriber('/base_pose_ground_truth', Odometry, self.statecallback)

		rcv_v = float(self.linear_v)
		ref_v = float(self.operations[2])
		
		if self.operations[0]:
			if abs(rcv_v - ref_v)/ref_v < thre:
				command.kappa = self.operations[3]
			else:
				ref_v = self.operations[1]
				pid = PID(P=1.2, I=10, D=0.1)
				pid.SetPoint = float(ref_v)
				pid.setSampleTime = rospy.Duration(1.0/20)
				pid.update(float(rcv_v))

				#print("pid output: "+ str(pid.output))
				command.fl_torque = pid.output
				command.fr_torque = pid.output
				command.rl_torque = pid.output
				command.rr_torque = pid.output
		else:
			command.fl_torque = self.operations[1]
			command.fr_torque = self.operations[2]
			command.rl_torque = self.operations[3]
			command.rr_torque = self.operations[4]
			command.kappa = self.operations[5]
			command.beta = self.operations[6]

		self.pub.publish(command)

	def pathControl(self):
		command = control_command()
		self.sub = rospy.Subscriber('rosout_agg', Log, self.velcallback)
		rospy.Subscriber('/base_pose_ground_truth', Odometry, self.statecallback)
		rcv_v = float(self.linear_v)

		# pid control for velocity
		pid = PID(P=2, I=1.2, D=0)
		pid.SetPoint = float(self.ref_v)
		pid.setSampleTime = rospy.Duration(1.0/20)
		pid.update(float(rcv_v))

		torque_thresh = -0.01
		output = pid.output
		if pid.output < torque_thresh:
			output = torque_thresh

		command.fl_torque = output
		command.fr_torque = output
		command.rl_torque = output
		command.rr_torque = output

		# pure pursuit for path follow
		lookahead = 1
		desire = 0
		Path = np.array(self.planPath)

		delta_x = Path[:, 0] - self.x
		delta_y = Path[:, 1] - self.y
		dist_list = delta_x*delta_x + delta_y*delta_y

		index = np.argmin(dist_list)
		if index + lookahead < len(Path) - 1:
			desire = index + lookahead
		else:
			desire = len(Path) - 1

		newPoint = planPath[desire]
		newPoint_x = newPoint[0] + 3
		newPoint_y = newPoint[1] - 12
		new_yaw = newPoint[2]/self.scale - self.yaw
		error_x = newPoint_x - self.x 
		error_y = newPoint_y - self.y
		dist = math.hypot(error_x, error_y)

		command.kappa = 2*math.sin(new_yaw)/dist
		command.beta = 0		#TODO: how to set beta in pure pursuit

		# stop the car if it approaches the end point
		if desire == len(Path) - 1 and self.pre_dist < dist:
			self.ref_v = 0
		else:
			self.ref_v = 1.2
		self.pre_dist = dist

		self.pub.publish(command)
		print("x: {5}, y: {6}, dist: {0}, desire_point: {1}, current_point {2}, kappa: {3}, velocity: {4}"
			.format(dist, desire, index, command.kappa, rcv_v, self.x, self.y))


if __name__ == '__main__':
	path_name = "path.dat"
	path_dir = "/home/el2425/catkin_ws/src/car_demo/car_demo/src/paths/"
	path_path = os.path.join(path_dir, path_name)
	rospy.init_node('rcv_controller', anonymous=True)

	print("Let's control your RCV")
	m_a = input("Choose automatic (1) or torque (0) control: ")
		
	if m_a:
		p_v = input("track path (1) or velocity (0): ")
		if p_v:
			planPath = []
			scale = 0.2
			with open(path_path) as f:
				for line in f:
					cur_data = [scale*float(x) for x in line.split(',')]
					planPath.append(cur_data)
			t = Translator(planPath=planPath, scale=scale)  
		else:
			vel = input("Input reference velocity: ")
			kappa = input("Input reference kappa: ")
			operations = [m_a, vel, kappa]
			t = Translator(operations=operations)
	else:
		fl_torque = input("Input the front-left wheel torque: ")
		fr_torque = input("Input the front-right wheel torque: ")
		rl_torque = input("Input the rear-left wheel torque: ")
		rr_torque = input("Input the rear-right wheel torque: ")
		kappa = input("Input the kappa: ")
		beta = input("Input the beta: ")
		operations = [m_a, fl_torque, fr_torque, rl_torque, rr_torque, kappa, beta]
		t = Translator(operations=operations)
	
	rospy.spin()

