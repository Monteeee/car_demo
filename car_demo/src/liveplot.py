#!/usr/bin/env python

import os
import numpy as np
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
import rospy
import math

def plot_xy(msg):
	global counter
	path_name = "path.dat"
	path_dir = "/home/el2425/catkin_ws/src/car_demo/car_demo/src/paths/"
	path_path = os.path.join(path_dir, path_name)
	x0 = 3.0
	y0 = -12.0
	scale = 0.2

	if counter == 0:
		planPath = []
		with open(path_path) as f:
			for line in f:
				cur_data = [scale*float(x) for x in line.split(',')]
				planPath.append(cur_data)
		Path = np.array(planPath)

		plt.plot(x0 + Path[:, 0], y0 + Path[:, 1], linewidth=2.0)
		plt.draw()
		plt.pause(1e-12)

	elif counter%100 == 0:
		cur_x = msg.pose.pose.position.x
		cur_y = msg.pose.pose.position.y
		cur_yaw = msg.pose.pose.orientation.z
		dx = math.sqrt(1.0/(1.0 + (np.tan(cur_yaw))**2))
		dy = dx*np.tan(cur_yaw)

		plt.plot(cur_x, cur_y, 'ro')
		ax = plt.axes()
		ax.arrow(cur_x, cur_y, dx, dy)
		plt.draw()
		plt.pause(1e-12)
	
	plt.title("Path name: " + path_name)
	plt.xlabel('x')
	plt.ylabel('y')
	counter += 1


if __name__ == '__main__':
	counter = 0
	rospy.init_node("plotter")
	rospy.Subscriber('/base_pose_ground_truth', Odometry, plot_xy)
	plt.ion()
	plt.show()
	rospy.spin()