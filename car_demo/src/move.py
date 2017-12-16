#!/usr/bin/env python

"""Main file for controlling RCV
"""

import os
import rospy
import math
import numpy as np
from matplotlib import pyplot as plt
import csv
import glob

import tf
from rcv_msgs.msg import control_command
from nav_msgs.msg import Odometry
from PID import PID 
from MPC import MPC
from PurePursuit import PurePursuit


class RCVControl:

	"""entry point to different control method
	"""

	def __init__(self, operations=None, ctrl_spec=None, planPath=None, ref_v=0.0):
		self.operations = operations
		self.ctrl_spec = ctrl_spec
		self.planPath = planPath
		self.linear_v = 0
		self.x0 = 0.0
		self.y0 = 0.0
		self.x = self.x0
		self.y = self.y0
		self.yaw = 0
		self.dist = 0
		self.pre_dist = 0
		self.ref_v = ref_v
		self.counter = 0
		self.vref = np.zeros((1, 60))
		self.pre_curv = 0.0
		self.pre_beta = 0.0
		self.log_path = '/home/el2425/catkin_ws/src/car_demo/car_demo/src/logs/'
		
		self.pub = rospy.Publisher('rcv_control_cmd', control_command, queue_size=1)
		self.state_sub = rospy.Subscriber('/base_pose_ground_truth', Odometry, self.state_callback)
		self.last_published_time = rospy.get_rostime()
		self.timer = rospy.Timer(rospy.Duration(1.0/20.0), self.timer_callback)
	
	def state_callback(self, data):
		if self.counter == 0:
			self.x0 = data.pose.pose.position.x
			self.y0 = data.pose.pose.position.y
		self.x = data.pose.pose.position.x
		self.y = data.pose.pose.position.y
		quaternion = (
			data.pose.pose.orientation.x,
			data.pose.pose.orientation.y,
			data.pose.pose.orientation.z,
			data.pose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.yaw = euler[2]
		self.linear_v = math.sqrt(data.twist.twist.linear.x**2 + data.twist.twist.linear.y**2)
				
	def timer_callback(self, event):
		if self.counter == 0:
			if self.ctrl_spec:
				self.log_path += 'mpc_ctrl/*.csv'
			else:
				self.log_path += 'pp_ctrl/*.csv'
			file_list = [int((os.path.basename(x)).replace('.csv', '')) for x in glob.glob(self.log_path)]
			self.log_path = self.log_path.replace('*', str(max(file_list) + 1)) if len(file_list) > 0 else self.log_path.replace('*', '1')
		self.counter += 1

		if self.last_published_time < rospy.get_rostime() + rospy.Duration(1.0/20):
			if self.operations is None:
				if self.ctrl_spec:
					self.pathControlMPC()
				else:
					self.pathControlPP()
			else:
				self.manControl()

	# interface takes torques, kappa, beta as input
	def manControl(self):
		command = control_command()
		
		command.fl_torque = self.operations[1]
		command.fr_torque = self.operations[2]
		command.rl_torque = self.operations[3]
		command.rr_torque = self.operations[4]
		command.kappa = self.operations[5]
		command.beta = self.operations[6]

		self.pub.publish(command)

	def pathControlPP(self):
		command = control_command()
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

		inte_path = self.interpolate(shape=self.planPath)
		purePursuit = PurePursuit(lookahead=20, planPath=inte_path)
		command.kappa = purePursuit.update([self.x - self.x0, self.y - self.y0, self.yaw])
		command.beta = 0		#TODO: how to set beta in pure pursuit

		# stop the car if it approaches the end point
		# if desire == len(Path) - 1 and self.pre_dist < dist:
		# 	self.ref_v = 0
		# else:
		# 	self.ref_v = 1.2
		# self.pre_dist = dist

		# save data as a log file
		with open(self.log_path, 'a') as newFile:
			newFileWriter = csv.writer(newFile)
			log_file = open(self.log_path)
			numline = len(log_file.readlines())
			if numline == 0:
				newFileWriter.writerow(['torque', 'kappa', 'velocity', 'x', 'y'])
			newFileWriter.writerow([command.fl_torque, command.kappa, rcv_v, self.x, self.y])

		self.pub.publish(command)
		#print("x: {5}, y: {6}, dist1: {7} dist: {0}, desire_point: {1}, current_point {2}, kappa: {3}, velocity: {4}"
		#	.format(dist, desire, index, command.kappa, rcv_v, self.x, self.y, dist_list[index]))

	def pathControlMPC(self):
		command = control_command()
		rcv_v = float(self.linear_v)
		ref_v_define = 1.3

		# pid control for velocity
		pid = PID(P=2, I=1.2, D=0)
		pid.SetPoint = float(ref_v_define)
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

		inte_path = self.interpolate(shape=self.planPath)
		Path = np.transpose(np.array(inte_path))
		length = Path.shape[0]
		delta_x = Path[:, 0] - self.x + self.x0
		delta_y = Path[:, 1] - self.y + self.y0
		dist_list = (delta_x*delta_x + delta_y*delta_y)**0.5
		index = np.argmin(dist_list)

		Hc, Hp, Ts_MPC = 15, 45, 1.0/20.0
		mpc = MPC(Hc=Hc, Hp=Hp, Ts_MPC=Ts_MPC)
		cur_state = np.array([self.x - self.x0, self.y - self.y0, self.yaw])

		if index+Hc+Hp < length:
			end = index+Hc+Hp
			true_path = Path[index:end, :]
		else:
			self.ref_v = 0
			true_path = Path[index:, :]

		true_path = np.reshape(true_path, (true_path.shape[0]*true_path.shape[1], 1))
		true_path = [float(i) for i in list(true_path)]
		#if self.counter < 2:
		#true_path = np.reshape(Path, (Path.shape[0]*Path.shape[1], 1))
		#true_path = [float(i) for i in list(true_path)]
		#else:
			#true_path = pathGen(Path[index:, :], self.vref)
			#print(true_path)
		curv, beta, vref = mpc.update(cur_state, true_path, self.pre_curv, self.pre_beta)
		command.kappa = curv
		command.beta = beta
		self.vref = vref
		self.ref_v = vref[0, 0]
		self.pre_curv = -curv
		self.pre_beta = beta

		# save data as a log file
		with open(self.log_path, 'a') as newFile:
			newFileWriter = csv.writer(newFile)
			log_file = open(self.log_path)
			numline = len(log_file.readlines())
			if numline == 0:
				newFileWriter.writerow(['torque', 'kappa', 'beta', 'velocity', 'x', 'y'])
			newFileWriter.writerow([command.fl_torque, command.kappa, command.beta, rcv_v, self.x, self.y])

		self.pub.publish(command)
		print('rcv_v: {0}'.format(rcv_v))
		#print("index: {0}, curv: {1}, beta: {2}, ref_v: {3}".format(index, curv, beta, ref_v))

	def interpolate(self, shape):
		route_x = []
		route_y = []
		points_per_meter = 15

		for index in range(1, len(shape)):
			dist_x = shape[index][0] - shape[index - 1][0]
			dist_y = shape[index][1] - shape[index - 1][1]
			len_temp = (dist_x**2 + dist_y**2)**0.5

			num_points = int(len_temp * float(points_per_meter))
			for num in range(0, num_points):
				temp_x = shape[index - 1][0] + num * dist_x / num_points
				temp_y = shape[index - 1][1] + num * dist_y / num_points

				route_x.append(temp_x)
				route_y.append(temp_y)

		if route_x == []:
			route_x.append(shape[0][0])
			route_y.append(shape[0][1])

		direction_list = []
		for index in range(1, len(route_x)):
			x = route_x[index] - route_x[index-1]
			y = route_y[index] - route_y[index-1]

			direction = np.arctan2(y, x)
			direction_list.append(direction)

		direction_list.append(0)

		return [route_x, route_y, direction_list]

def pathGen(path, vref):
	total_distance_covered = 0
	delta_t = 1.0/20.0
	dist_stack, new_path = [], []

	for i in range(path.shape[0]-1):
		cur_dist = math.hypot((path[i,0] - path[i+1,0]), (path[i,1] - path[i+1,1]))
		dist_stack.append(cur_dist)

	cum_dist = list(np.cumsum(np.array(dist_stack)))
	total_distance = cum_dist[-1]

	for idx in range(vref.shape[1]): 
		distance_covered = vref[0, idx] * delta_t
		total_distance_covered += distance_covered

		if total_distance_covered >= total_distance:
			total_distance_covered = total_distance
			new_path.append(path[-1,0])
			new_path.append(path[-1,1])
			new_path.append(path[-1,2])
		else:
			index = 0
			while total_distance_covered >= cum_dist[index]:
				index += 1

			distance_left = total_distance_covered - cum_dist[index]
			state_x = path[index,0]
			state_y = path[index,1]
			state_yaw = path[index,2]

			new_path.append(float(state_x + distance_left * np.cos(state_yaw)))
			new_path.append(float(state_y + distance_left * np.sin(state_yaw)))
			new_path.append(float(state_yaw))

	return new_path


if __name__ == '__main__':
	path_name = "path.dat"
	path_dir = "/home/el2425/catkin_ws/src/car_demo/car_demo/src/paths/"
	path_path = os.path.join(path_dir, path_name)
	rospy.init_node('rcv_controller', anonymous=True)

	print("Let's control your RCV")
	m_a = input("Choose automatic (1) or torque (0) control: ")
		
	if m_a:
		ctrl_spec = input("MPC (1) or Pure Pursuit (0): ")
		planPath = []
		scale_x = 1
		scale_y = 1
		with open(path_path) as f:
			for line in f:
				cur_data = [float(x) for x in line.split(',')]
				cur_data[0] *= scale_x
				cur_data[0] -= 130.0
				cur_data[1] *= scale_y
				planPath.append(cur_data)
		if ctrl_spec:
			t = RCVControl(ctrl_spec=ctrl_spec, planPath=planPath)
		else:
			ref_v = input("Input reference velocity: ")
			t = RCVControl(ctrl_spec=ctrl_spec, planPath=planPath, ref_v=ref_v)  
	else:
		fl_torque = input("Input the front-left wheel torque: ")
		fr_torque = input("Input the front-right wheel torque: ")
		rl_torque = input("Input the rear-left wheel torque: ")
		rr_torque = input("Input the rear-right wheel torque: ")
		kappa = input("Input the kappa: ")
		beta = input("Input the beta: ")
		operations = [m_a, fl_torque, fr_torque, rl_torque, rr_torque, kappa, beta]
		t = RCVControl(operations=operations)
	
	rospy.spin()

