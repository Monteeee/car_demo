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
from rcv_msgs.msg import control_command
from rosgraph_msgs.msg import Log
from nav_msgs.msg import Odometry
from PID import PID 
#from scipy.spatial.distance import cdist
import math
import numpy as np



STEERING_AXIS = 0
THROTTLE_AXIS = 4
thre = 0.2


class Translator:
	def __init__(self, operations=None, planPath=None):
		#self.pub = rospy.Publisher('rcv_control_cmd', Control, queue_size=1)
		self.pub = rospy.Publisher('rcv_control_cmd', control_command, queue_size=1)
		self.last_published_time = rospy.get_rostime()
		self.operations = operations
		self.planPath = planPath
		self.timer = rospy.Timer(rospy.Duration(1.0/20.0), self.timer_callback)
		self.linear_v = 0
		self.index = 0
		self.x = 3
		self.y = -12
		self.yaw = 0
		self.dist = 0
		self.ref_v = 3
	

	def callback(self, data):
		self.linear_v = data.msg
	
	def statecallback(self, data):
		self.x = data.pose.pose.position.x
		self.y = data.pose.pose.position.y
		self.yaw = data.pose.pose.orientation.z
				
	def timer_callback(self, event):
		if self.last_published_time < rospy.get_rostime() + rospy.Duration(1.0/20):
			self.pathControl()
			#self.move_new()

	def move(self):
		command = Control()
		self.sub = rospy.Subscriber('rosout_agg', Log, self.callback)
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

	def move_new(self):
		command = control_command()
		self.sub = rospy.Subscriber('rosout_agg', Log, self.callback)
		rospy.Subscriber('/base_pose_ground_truth', Odometry, self.statecallback)

		rcv_v = float(self.linear_v)
		ref_v = float(self.operations[2])
		
		if self.operations[0]:
			if abs(rcv_v - ref_v)/ref_v < thre:
				command.kappa = self.operations[3]
			else:
				ref_v = self.operations[2]
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
		#kappa = 0
		command = control_command()
		self.sub = rospy.Subscriber('rosout_agg', Log, self.callback)
		rospy.Subscriber('/base_pose_ground_truth', Odometry, self.statecallback)
		rcv_v = float(self.linear_v)

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

		lookahead = 2
		desire = 0
		Path = np.array(self.planPath)

		delta_x = Path[:,0] - self.x
		delta_y = Path[:,1] - self.y

		dist = delta_x*delta_x + delta_y*delta_y

		index = np.argmin(dist)
		if index + lookahead < len(Path) -1:
			desire = index + lookahead
		else:
			desire = len(Path) - 1

		newPoint = planPath[desire]
		newPoint_x = newPoint[0] + 3
		newPoint_y = newPoint[1] - 12
		new_yaw = 5*newPoint[2] - self.yaw
		error_x = newPoint_x - self.x 
		error_y = newPoint_y - self.y
		distance = math.hypot(error_x, error_y)

		# if distance > 2:
		# 	if abs(delta_y) < 0.5:
		# 		kappa = 0

		command.kappa = 2*math.sin(new_yaw)/distance
		command.beta = 0

		self.pub.publish(command)
		print("y: {0}, new_y: {1}, dist: {2}, desire_point: {3}, current_point {4}, kappa: {5}".format(self.y, newPoint_y, distance, desire, index, command.kappa))
		#print("yaw: {2}, dist: {0}, kappa: {1}".format(distance, command.kappa, self.yaw))
		

		# desire_x = desire[0]
		# desire_y = desire[1]
		# torque = 1
		# beta = 0
		# kappa = 1
		# m_a = 1
		# fl_torque = 1
		# fr_torque = 1
		# rl_torque = 1
		# rr_torque = 1

		# command.fl_torque = self.operations[1]
		# command.fr_torque = self.operations[2]
		# command.rl_torque = self.operations[3]
		# command.rr_torque = self.operations[4]
		# command.kappa = self.operations[5]
		# command.beta = self.operations[6]
		# dist = math.hypot((desire_x-self.x),(desire_y-self.y))
		# self.pub.publish(command)


if __name__ == '__main__':
	path = "/home/el2425/catkin_ws/src/simulation_nodes/fake_planning/path_from_file_planner/data/path.dat"
	rospy.init_node('rcv_controller', anonymous=True)

	print("Let's control your RCV")
	m_a = input("Choose automatic (1) or torque (0) control: ")
		
	if m_a:
		p_v = input("track path (1) or velocity (0): ")
		if p_v:
			planPath = []	
			with open(path) as f:
				for line in f:
					cur_data = [0.2*float(x) for x in line.split(',')]
					planPath.append(cur_data)
			#print(planPath)
			t = Translator(planPath=planPath)  
		else:
			vel = input("Input reference velocity: ")
			kappa = input("Input reference kappa: ")
			operations = [m_a, p_v, vel, kappa]
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

