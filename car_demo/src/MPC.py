#!/usr/bin/python
#
# Code made by Feiyang Liu(liuf@kth.se), Tianxiao Zhao(tzh@kth.se)

"""A simple implementation of MPC using cvxopt as solver
"""


import numpy as np
from math import hypot
import math
from cvxopt import matrix, solvers


class MPC:

	"""MPC Controller
	"""

	def __init__(self, Hc=5, Hp=15, Ts_MPC=0.09):
		self.Hc = Hc
		self.Hp = Hp
		self.Ts_MPC = Ts_MPC

	def update(self, z, true_path):
		# Dimentions of input variables:
		# z = (x,y,yaw)  #(1x3) 
		# true_path: (x0, y0, yaw0, x1, y1, yaw1,...)  #(3N x 1)

		w_x = 10
		w_y = 50
		w_yaw = 0
		w_beta = 100
		w_curv = 1000

		a_flPlane = 0.9728
		b_flPlane = 0.9866
		c_flPlane = 0.0079

		a_frPlane = 1.0331
		b_frPlane = 1.0090
		c_frPlane = -0.0079

		a_rlPlane = -0.9915
		b_rlPlane = 0.9929
		c_rlPlane = -0.0078

		a_rrPlane = -1.0178
		b_rrPlane = 1.0050
		c_rrPlane = -0.0066

		maxdeltafl = 0.2255
		mindeltafl = -0.2480

		maxdeltafr = 0.2640
		mindeltafr = -0.2910

		maxdeltarl = 0.3150
		mindeltarl = -0.2235

		maxdeltarr = 0.2330
		mindeltarr = -0.2937

		maxdeltarate = 0.3700
		mindeltarate = -0.3700

		global init_flag, prev_curv, prev_beta
		#if init_flag == []:
		init_flag = 1
		prev_curv = 0
		prev_beta = 0
		
		#need to modify how to load the path
		true_path = np.array(true_path)
		length = len(true_path)
		true_path = np.reshape(true_path,(int(length/3),3))

		# Translate path from global (X,Y,YAW) to local (x,y,yaw) coordinate
		zref = np.zeros((1, 3 * (self.Hp + self.Hc)))
		curv_beta_ref = np.zeros((1, 2 * (self.Hp + self.Hc)))
		traj_crab = np.zeros((self.Hp + self.Hc, 2))
		vref = np.zeros((1, self.Hp + self.Hc))

		rot = np.array([[np.cos(z[2]), np.sin(z[2])],[-np.sin(z[2]), np.cos(z[2])]])
		for i in range(self.Hp + self.Hc - 1):
			z_temp = true_path[i, :]
			xy_temp = np.dot(rot, np.array([[z_temp[0]-z[0]],[z_temp[1]-z[1]]]))
			yaw_temp = z_temp[2] - z[2]
			zref[0,3 * i] = xy_temp[0,:]
			zref[0,3 * i + 1] = xy_temp[1,:]
			zref[0,3 * i + 2] = yaw_temp

		# z0 = [0.0, 0.0, 0.0]
		# z0.extend(list(zref[0, :]))
		# zref = np.array([z0])

		# Calculating curvature (kappa) and crabbing (beta) reference
		for i in range(self.Hp + self.Hc - 1):
			xy_crab = np.dot(np.array([[np.cos(zref[0, 3*i+2]), np.sin(zref[0, 3*i+2])], [-np.sin(zref[0, 3*i+2]), np.cos(zref[0, 3*i+2])]]), np.array([[zref[0, 3*i+3] - zref[0, 3*i]], [zref[0, 3*i+4] - zref[0, 3*i+1]]]))
			traj_crab[i, 0] = xy_crab[0,0]
			traj_crab[i, 1] = xy_crab[1,0]

		for j in range(self.Hp + self.Hc - 1):
			curv_beta_ref[0,2*j] = 0.5 * hypot((zref[0,3*j]-zref[0,3*j+3]), (zref[0,3*j+1] - zref[0,3*j+4])) / np.sin(0.5*(zref[0,3*j+5] - zref[0,3*j+2]))
			curv_beta_ref[0,2*j] = 1.0/curv_beta_ref[0,2*j] if curv_beta_ref[0,2*j] != 0.0 else 10.0
			if abs(traj_crab[j+1,0] - traj_crab[j,0]) < 0.0001:
				curv_beta_ref[0,2*j+1] = 0.0
			else:
				curv_beta_ref[0,2*j+1] = math.atan2(traj_crab[j+1,0] - traj_crab[j,0], traj_crab[j+1, 1] - traj_crab[j,1])

		# Calculating velocity reference vref
		for i in range(self.Hp + self.Hc - 1):
			vref[0,i] = hypot((zref[0,3*i+3] - zref[0,3*i]), (zref[0,3*i+4] - zref[0,3*i+1])) / self.Ts_MPC

		z_dif = np.array([[-zref[0,0]], [-zref[0,1]], [-zref[0,2]]])

		yaw_aux = zref[0,np.arange(2,3*(self.Hp+self.Hc),3)] - zref[0,2]

		#Constraints Initialization
		SteeringPlanes = np.array([[a_flPlane],[b_flPlane], [a_frPlane], [b_frPlane], [a_rlPlane], [b_rlPlane], [a_rrPlane], [b_rrPlane]])

		ubfl = maxdeltafl - (curv_beta_ref[0,np.arange(0,2*self.Hc,2)] * a_flPlane + curv_beta_ref[0,np.arange(1,2*self.Hc,2)] * b_flPlane + c_flPlane)
		lbfl = mindeltafl - (curv_beta_ref[0,np.arange(0,2*self.Hc,2)] * a_flPlane + curv_beta_ref[0,np.arange(1,2*self.Hc,2)] * b_flPlane + c_flPlane)

		ubfr = maxdeltafr - (curv_beta_ref[0,np.arange(0,2*self.Hc,2)] * a_frPlane + curv_beta_ref[0,np.arange(1,2*self.Hc,2)] * b_frPlane + c_frPlane)
		lbfr = mindeltafr - (curv_beta_ref[0,np.arange(0,2*self.Hc,2)] * a_frPlane + curv_beta_ref[0,np.arange(1,2*self.Hc,2)] * b_frPlane + c_frPlane)

		ubrl = maxdeltarl - (curv_beta_ref[0,np.arange(0,2*self.Hc,2)] * a_rlPlane + curv_beta_ref[0,np.arange(1,2*self.Hc,2)] * b_rlPlane + c_rlPlane)
		lbrl = mindeltarl - (curv_beta_ref[0,np.arange(0,2*self.Hc,2)] * a_rlPlane + curv_beta_ref[0,np.arange(1,2*self.Hc,2)] * b_rlPlane + c_rlPlane)

		ubrr = maxdeltarr - (curv_beta_ref[0,np.arange(0,2*self.Hc,2)] * a_rrPlane + curv_beta_ref[0,np.arange(1,2*self.Hc,2)] * b_rrPlane + c_rrPlane)
		lbrr = mindeltarr - (curv_beta_ref[0,np.arange(0,2*self.Hc,2)] * a_rrPlane + curv_beta_ref[0,np.arange(1,2*self.Hc,2)] * b_rrPlane + c_rrPlane)

		ubflRate = np.zeros((self.Hc, 1))
		lbflRate = np.zeros((self.Hc, 1))
		ubflRate[0,0] = maxdeltarate * self.Ts_MPC + (prev_curv * a_flPlane + prev_beta * b_flPlane) - (curv_beta_ref[0,0] * a_flPlane + curv_beta_ref[0,1] * b_flPlane)
		lbflRate[0,0] = mindeltarate * self.Ts_MPC + (prev_curv * a_flPlane + prev_beta * b_flPlane) - (curv_beta_ref[0,0] * a_flPlane + curv_beta_ref[0,1] * b_flPlane)

		ubfrRate = np.zeros((self.Hc, 1))
		lbfrRate = np.zeros((self.Hc, 1))
		ubfrRate[0,0] = maxdeltarate * self.Ts_MPC + (prev_curv * a_frPlane + prev_beta * b_frPlane) - (curv_beta_ref[0,0] * a_frPlane + curv_beta_ref[0,1] * b_frPlane)
		lbfrRate[0,0] = mindeltarate * self.Ts_MPC + (prev_curv * a_frPlane + prev_beta * b_frPlane) - (curv_beta_ref[0,0] * a_frPlane + curv_beta_ref[0,1] * b_frPlane)

		ubrlRate = np.zeros((self.Hc, 1))
		lbrlRate = np.zeros((self.Hc, 1))
		ubrlRate[0,0] = maxdeltarate * self.Ts_MPC + (prev_curv * a_rlPlane + prev_beta * b_rlPlane) - (curv_beta_ref[0,0] * a_rlPlane + curv_beta_ref[0,1] * b_rlPlane)
		lbrlRate[0,0] = mindeltarate * self.Ts_MPC + (prev_curv * a_rlPlane + prev_beta * b_rlPlane) - (curv_beta_ref[0,0] * a_rlPlane + curv_beta_ref[0,1] * b_rlPlane)

		ubrrRate = np.zeros((self.Hc, 1))
		lbrrRate = np.zeros((self.Hc, 1))
		ubrrRate[0,0] = maxdeltarate * self.Ts_MPC + (prev_curv * a_rrPlane + prev_beta * b_rrPlane) - (curv_beta_ref[0,0] * a_rrPlane + curv_beta_ref[0,1] * b_rrPlane)
		lbrrRate[0,0] = mindeltarate * self.Ts_MPC + (prev_curv * a_rrPlane + prev_beta * b_rrPlane) - (curv_beta_ref[0,0] * a_rrPlane + curv_beta_ref[0,1] * b_rrPlane)

		#Matrices Init
		R = np.zeros((2 * self.Hc, 2 * self.Hc))
		Q = np.zeros((3 * (self.Hc + self.Hp),3 * (self.Hc + self.Hp)))
		A = np.zeros((3 * (self.Hc + self.Hp), 3))
		Ai = np.zeros(((self.Hc + self.Hp), 3, 3))
		B = np.zeros((3 * (self.Hc + self.Hp), 2 * self.Hc))
		Bi = np.zeros(((self.Hc + self.Hp), 3, 2))

		#Matrices fill-up for control horizon
		Ai[0, :, :] = np.array([[1, 0, self.Ts_MPC * (-np.sin(yaw_aux[0] + curv_beta_ref[0,1]) * vref[0,0])], [0, 1, self.Ts_MPC * (np.cos(yaw_aux[0] + curv_beta_ref[0,1]) * vref[0,0])], [0, 0, 1]])
		Bi[0, :, :] = np.array([[0, self.Ts_MPC * (-np.sin(yaw_aux[0] + curv_beta_ref[0,1]) * vref[0,0])], [0, self.Ts_MPC * (np.cos(yaw_aux[0] + curv_beta_ref[0,1]) * vref[0,0])], [self.Ts_MPC * vref[0,0], 0]])

		A[np.arange(0,3,1), :] = Ai[0,:,:]
		B[np.arange(0,3,1), 0] = Bi[0,:,0]
		B[np.arange(0,3,1), 1] = Bi[0,:,1]	
	
		for i in range(self.Hc-1):
			#Constraints Initialization
			ubflRate[i+1, 0] = maxdeltarate * self.Ts_MPC + (curv_beta_ref[0,2*i] * a_flPlane + curv_beta_ref[0,2*i+1] * b_flPlane) - (curv_beta_ref[0,2 * (i + 1)] * a_flPlane + curv_beta_ref[0,2 * (i + 1)+1] * b_flPlane)
			lbflRate[i+1, 0] = mindeltarate * self.Ts_MPC + (curv_beta_ref[0,2*i] * a_flPlane + curv_beta_ref[0,2*i+1] * b_flPlane) - (curv_beta_ref[0,2 * (i + 1)] * a_flPlane + curv_beta_ref[0,2 * (i + 1)+1] * b_flPlane)

			ubfrRate[i+1, 0] = maxdeltarate * self.Ts_MPC + (curv_beta_ref[0,2*i] * a_frPlane + curv_beta_ref[0,2*i+1] * b_frPlane) - (curv_beta_ref[0,2 * (i + 1)] * a_frPlane + curv_beta_ref[0,2 * (i + 1)+1] * b_frPlane)
			lbfrRate[i+1, 0] = mindeltarate * self.Ts_MPC + (curv_beta_ref[0,2*i] * a_frPlane + curv_beta_ref[0,2*i+1] * b_frPlane) - (curv_beta_ref[0,2 * (i + 1)] * a_frPlane + curv_beta_ref[0,2 * (i + 1)+1] * b_frPlane)

			ubrlRate[i+1, 0] = maxdeltarate * self.Ts_MPC + (curv_beta_ref[0,2*i] * a_rlPlane + curv_beta_ref[0,2*i+1] * b_rlPlane) - (curv_beta_ref[0,2 * (i + 1)] * a_rlPlane + curv_beta_ref[0,2 * (i + 1)+1] * b_rlPlane)
			lbrlRate[i+1, 0] = mindeltarate * self.Ts_MPC + (curv_beta_ref[0,2*i] * a_rlPlane + curv_beta_ref[0,2*i+1] * b_rlPlane) - (curv_beta_ref[0,2 * (i + 1)] * a_rlPlane + curv_beta_ref[0,2 * (i + 1)+1] * b_rlPlane)

			ubrrRate[i+1, 0] = maxdeltarate * self.Ts_MPC + (curv_beta_ref[0,2*i] * a_rrPlane + curv_beta_ref[0,2*i+1] * b_rrPlane) - (curv_beta_ref[0,2 * (i + 1)] * a_rrPlane + curv_beta_ref[0,2 * (i + 1)+1] * b_rrPlane)
			lbrrRate[i+1, 0] = mindeltarate * self.Ts_MPC + (curv_beta_ref[0,2*i] * a_rrPlane + curv_beta_ref[0,2*i+1] * b_rrPlane) - (curv_beta_ref[0,2 * (i + 1)] * a_rrPlane + curv_beta_ref[0,2 * (i + 1)+1] * b_rrPlane)

			#Matrices fill-up for control horizon

			Ai[i+1, :, :] = np.array([[1, 0, self.Ts_MPC * (-np.sin(yaw_aux[i+1] + curv_beta_ref[0, 2*i+3]) * vref[0,i+1])],[0, 1, self.Ts_MPC * (np.cos(yaw_aux[i+1] + curv_beta_ref[0,2*i+3]) * vref[0,i+1])], [0, 0, 1]])
			Bi[i+1, :, :] = np.array([[0, self.Ts_MPC * (-np.sin(yaw_aux[i+1] + curv_beta_ref[0, 2*i+3]) * vref[0,i+1])],[0, self.Ts_MPC * (np.cos(yaw_aux[i+1] + curv_beta_ref[0,2*i+3]) * vref[0,i+1])],[self.Ts_MPC * vref[0,i+1], 0]])

			A[np.arange(3*i+3,3*i+6,1), :] = np.dot(Ai[i + 1,:,:], A[np.arange(3*i,3*i+3,1),:])

			for j in range(i):
				midB = np.zeros((3,2))
				midB[:, 0] = B[np.arange(3*i,3*i+3,1), 2*j]
				midB[:, 1] = B[np.arange(3*i,3*i+3,1), 2*j + 1]
				finalB = np.dot(Ai[i+1,:,:], midB)
				B[np.arange(3*i+3,3*i+6,1), 2*j] = finalB[:, 0]
				B[np.arange(3*i+3,3*i+6,1), 2*j+1] = finalB[:, 1]
				#B[np.arange(3*i+3,3*i+6,1), np.arange(2*j,2*j+1,1)] = np.dot(Ai[i+1,:,:], B[np.arange(3*i,3*i+3,1), np.arange(2*j,2*j+2,1)])

			B[np.arange(3*i+3, 3*i+6, 1), 2*i+2] = Bi[i+1, :, 0]
			B[np.arange(3*i+3, 3*i+6, 1), 2*i+3] = Bi[i+1, :, 1]

			Q[3*i, 3*i] = w_x
			Q[3*i+1, 3*i+1] = w_y
			Q[3*i+2, 3*i+2] = w_yaw
			R[2*i+1, 2*i+1] = w_beta
			R[2*i, 2*i] = w_curv

		index = self.Hc - 1
		Q[3*index, 3*index] = w_x
		Q[3*index+1, 3*index+1] = w_y
		Q[3*index+2, 3*index+2] = w_yaw
		R[2*index+1, 2*index+1] = w_beta
		R[2*index, 2*index] = w_curv

		#Matrices fill-up for prediction horizon
		for i in range(self.Hp):
			Ai[i+self.Hc, :, :] = np.array([[1, 0, self.Ts_MPC * (-np.sin(yaw_aux[i+self.Hc] + curv_beta_ref[0, 2*self.Hc+2*i-1]) * vref[0,i+self.Hc])], [0, 1, self.Ts_MPC * (np.cos(yaw_aux[i + self.Hc] + curv_beta_ref[0,2*self.Hc + 2*i+1]) * vref[0,i + self.Hc])],[0, 0, 1]])
			Bi[i+self.Hc, :, :] = np.array([[0, self.Ts_MPC * (-np.sin(yaw_aux[i + self.Hc] + curv_beta_ref[0, 2*self.Hc +2*i+1]) * vref[0,i+self.Hc])], [0, self.Ts_MPC * (np.cos(yaw_aux[i + self.Hc] + curv_beta_ref[0, 2*self.Hc + 2*i+1]) * vref[0,i + self.Hc])], [self.Ts_MPC * vref[0,i + self.Hc], 0]])

			A[np.arange(i*3+3*self.Hc,3*i+3+3*self.Hc,1), :] = np.dot(Ai[i+self.Hc,:,:], A[np.arange(3*i-3+3*self.Hc,3*i+3*self.Hc,1),:])
			for j in range(self.Hc-1):
				midB = np.zeros((3,2))
				midB[:, 0] = B[np.arange(3*i-3+3*self.Hc,3*i+3*self.Hc,1), 2*j]
				midB[:, 1] = B[np.arange(3*i-3+3*self.Hc,3*i+3*self.Hc,1), 2*j+1]
				finalB = np.dot(Ai[i+self.Hc,:,:], midB)
				B[np.arange(i*3 + (3*self.Hc), i*3+3 + (3*self.Hc), 1), 2*j] = finalB[:, 0]
				B[np.arange(i*3 + (3*self.Hc), i*3+3 + (3*self.Hc), 1), 2*j+1] = finalB[:, 1]
				#B[np.arange(i*3+3*self.Hc,3*i+3+3*self.Hc,1), np.arange(2*j,2*j+2,1)] = np.dot(Ai[i+self.Hc,:,:],B[np.arange(3*i-3+3*self.Hc,3*i+3*self.Hc,1),np.arange(2*j,2*j+2,1)])
			
			midB = np.zeros((3,2))
			midB[:, 0] = B[np.arange(3*i-3+3*self.Hc,3*i+3*self.Hc,1), 2*self.Hc - 2]
			midB[:, 1] = B[np.arange(3*i-3+3*self.Hc,3*i+3*self.Hc,1), 2*self.Hc - 1]
			finalB = np.dot(Ai[i+self.Hc, :, :], midB)
			B[np.arange(i*3 + (3*self.Hc), i*3+3 + (3*self.Hc), 1), 2*self.Hc - 2] = finalB[:, 0] + Bi[i+self.Hc, :, 0]
			B[np.arange(i*3 + (3*self.Hc), i*3+3 + (3*self.Hc), 1), 2*self.Hc - 1] = finalB[:, 1] + Bi[i+self.Hc, :, 1]			
			#B[np.arange(i*3+(3*self.Hc),i*3+3+(3*self.Hc),1),np.arange(2*self.Hc,2*self.Hc+2,1)] = np.dot(Ai[i+self.Hc,:,:],B[np.arange(3*i-3+3*self.Hc,3*i+3*self.Hc,1),np.arange(2*self.Hc-2,2*self.Hc,1)])
			Q[3 * i + (3 * self.Hc), 3 * i + (3 * self.Hc)] = w_x
			Q[3 * i + 1 + (3 * self.Hc), 3 * i + 1 + (3 * self.Hc)] = w_y
			Q[3 * i + 2 + (3 * self.Hc), 3 * i + 2 + (3 * self.Hc)] = w_yaw

		Hess = np.dot(np.dot(np.transpose(B), Q), B) + R
		Hess = (Hess + np.transpose(Hess)) / 2
		lin = 2 * np.dot(np.dot(np.dot(np.transpose(B), Q), A),z_dif) 
		#lin = 2 * (np.dot(np.dot(np.dot(np.transpose(B), Q), A),z_dif) + np.dot(R,np.transpose(curv_beta_ref[0,np.arange(0,2*self.Hc,1)])))

		sol = self.cvxsolve(Hess, lin, ubfl, lbfl, ubfr, lbfr, ubrl, lbrl, ubrr, lbrr, ubflRate, lbflRate, ubfrRate, lbfrRate, ubrlRate, lbrlRate, ubrrRate, lbrrRate, self.Hc)
		res_dif = sol['x']
		
		curv_beta_Hc = curv_beta_ref[0, np.arange(0,2*self.Hc,1)] + res_dif

		curv_out = curv_beta_Hc[0, 0]
		beta_out = curv_beta_Hc[0, 1]
		v_out = vref[0,0]


		if (np.isnan(curv_out) or np.isnan(beta_out)):
			beta_out = 0.0
			curv_out = 0.0
			v_out = vref[0,0]

		prev_curv = curv_out
		prev_beta = beta_out

		return curv_out, beta_out, v_out


	def cvxsolve(self, Hess, lin, ubfl, lbfl, ubfr, lbfr, ubrl, lbrl, ubrr, lbrr, ubflRate, lbflRate, ubfrRate, lbfrRate, ubrlRate, lbrlRate, ubrrRate, lbrrRate, Hc):

		a_flPlane = 0.9728
		b_flPlane = 0.9866
		c_flPlane = 0.0079

		a_frPlane = 1.0331
		b_frPlane = 1.0090
		c_frPlane = -0.0079

		a_rlPlane = -0.9915
		b_rlPlane = 0.9929
		c_rlPlane = -0.0078

		a_rrPlane = -1.0178
		b_rrPlane = 1.0050
		c_rrPlane = -0.0066

		steerplane = matrix([a_flPlane, b_flPlane, a_frPlane, b_frPlane, a_rlPlane, b_rlPlane, a_rrPlane, b_rrPlane])
		plane = matrix(steerplane,(2,4))

		A = matrix(0.0, (8*Hc, 2*Hc))
		for i in range(4):
			for j in range(Hc):
				A[i*2*Hc + j, 2*j] = plane[0, i]
				A[i*2*Hc + j, 2*j + 1] = plane[1, i]
				A[i*2*Hc + j + Hc, 2*j] = -plane[0, i]
				A[i*2*Hc + j + Hc, 2*j + 1] = -plane[1, i]


		bound = np.array([ubfl, -lbfl, ubfr, -lbfr, ubrl, -lbrl, ubrr, -lbrr])
		B = matrix(0.0,(8*Hc, 1))
		for i in range(8):
			B[(i*Hc):(i*Hc+Hc),0] = matrix(bound[i,:],(Hc,1))

		C = matrix(0.0,(8*Hc, 2*Hc))
		for j in range(4):
			C[j*2*Hc, 0] = plane[0, j]
			C[j*2*Hc, 1] = plane[1, j]
			C[j*2*Hc + Hc, 0] = -plane[0, j]
			C[j*2*Hc + Hc, 1] = -plane[1, j]

		for i in range(4):
			for j in range(Hc - 1):
				C[i*2*Hc + j + 1, 2*j] = -plane[0, i]
				C[i*2*Hc + j + 1, 2*j + 1] = -plane[1, i]
				C[i*2*Hc + j + 1, 2*j + 2] = plane[0, i]
				C[i*2*Hc + j + 1, 2*j + 3] = plane[1, i]

				C[i*2*Hc + j + 1 + Hc, 2*j] = plane[0, i]
				C[i*2*Hc + j + 1 + Hc, 2*j + 1] = plane[1, i]
				C[i*2*Hc + j + 1 + Hc, 2*j + 2] = -plane[0, i]
				C[i*2*Hc + j + 1 + Hc, 2*j + 3] = -plane[1, i]

		D = matrix(0.0,(8*Hc, 1))
		boundplane = np.array([ubflRate, -lbflRate, ubfrRate, -lbfrRate, ubrlRate, -lbrlRate, ubrrRate, -lbrrRate])
		#print(boundplane)
		for i in range(8):
			D[(i*Hc):(i*Hc+Hc),0] = matrix(boundplane[i,:,:])
		#print(D)


		constrain1 = matrix(0.0,(16*Hc, 2*Hc))
		constrain1[0:8*Hc, :] = A
		constrain1[8*Hc:16*Hc, :] = C
		constrain2 = matrix([[B],[D]],(16*Hc, 1))

		Hess = matrix(Hess)
		lin = matrix(lin)

		solvers.options['show_progress'] = False
		output = solvers.qp(Hess, lin, constrain1, constrain2)

		return output
