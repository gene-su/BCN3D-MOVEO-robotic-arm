"""Piper solution for certain pose of BCN3D MOVEO robot arm"""

from sympy import *
import numpy as np

def r2d(radius):
	if radius == None:
		return None
	return radius/np.pi*180

def d2r(degree):
	return degree/180*np.pi

def check_zero(list_):
	for i in range(len(list_)):
		if list_[i] < 1e-10:
			list_[i] = 0

def check_pi(list_):
	for i in range(len(list_)):
		if list_[i] == pi:
			list_[i] = np.pi

def degree_trans(x):
	if type(x) == list:
		for i in range(len(x)):
			if x[i] > 180:
				x[i] = x[i] - 360
			elif x[i] < -180:
				x[i] = x[i] + 360
	else:
		if x > 180:
			x = x - 360
		elif x < -180:
			x = x + 360
	return x

def IK(goal):
	"""
	Inputs: list of x-y-z offset
	Outputs: list of angles for joints to move
	"""

	# Variable setting
	al_0 = Symbol('al_0')
	a_0 = Symbol('a_0')
	d_1 = Symbol('d_1')
	t_1 = Symbol('t_1')
	al_1 = Symbol('al_1')
	a_1 = Symbol('a_1')
	d_2 = Symbol('d_2')
	t_2 = Symbol('t_2')
	al_2 = Symbol('al_2')
	a_2 = Symbol('a_2')
	d_3 = Symbol('d_3')
	t_3 = Symbol('t_3')
	al_3 = Symbol('al_3')
	a_3 = Symbol('a_3')
	d_4 = Symbol('d_4')
	t_4 = Symbol('t_4')
	al_4 = Symbol('al_4')
	a_4 = Symbol('a_4')
	d_5 = Symbol('d_5')
	t_5 = Symbol('t_5')
	al_5 = Symbol('al_5')
	a_5 = Symbol('a_5')
	d_6 = Symbol('d_6')
	t_6 = Symbol('t_6')

	offset = goal # x-y-z offset

	DH_table = {al_0: 0, al_1: -pi/2, al_2: 0, al_3: pi/2, al_4: -pi/2, al_5: pi/2,
	            a_0: 0, a_1: 0, a_2: 22.4, a_3: 0, a_4: 0, a_5: 0,
	            d_1: 0, d_2: 0, d_3: 0, d_4: 23, d_5: 0, d_6: 0}

	T_WB = Matrix([[offset[0]/np.sqrt(offset[0]**2 + offset[1]**2), -offset[1]/np.sqrt(offset[0]**2 + offset[1]**2), 0, offset[0]],
	            [offset[1]/np.sqrt(offset[0]**2 + offset[1]**2), offset[0]/np.sqrt(offset[0]**2 + offset[1]**2), 0, offset[1]],
	            [0, 0, 1, offset[2]],
	            [0, 0, 0, 1]])

	T_W0 = Matrix([[1, 0, 0, 0],
	        	   [0, 1, 0, 0],
	               [0, 0, 1, 23],
	               [0, 0, 0, 1]])

	T_6B = Matrix([[-1, 0, 0, 0],
	        	   [0, 1, 0, 0],
	               [0, 0, -1, 15],
	               [0, 0, 0, 1]])

	T_06 = T_W0.inv() * T_WB * T_6B.inv()
	T_06 = np.array(T_06).astype(np.float64)
	T_01 = Matrix([[cos(t_1), -sin(t_1), 0, a_0],
					[sin(t_1)*cos(al_0), cos(t_1)*cos(al_0), -sin(al_0), -sin(al_0)*d_1],
	                 [sin(t_1)*sin(al_0), cos(t_1)*sin(al_0), cos(al_0), cos(al_0)*d_1],
	                 [0, 0, 0, 1]])

	T_12 = Matrix([[cos(-pi/2+t_2), -sin(-pi/2+t_2), 0, a_1],
	               [sin(-pi/2+t_2)*cos(al_1), cos(-pi/2+t_2)*cos(al_1), -sin(al_1), -sin(al_1)*d_2],
	               [sin(-pi/2+t_2)*sin(al_1), cos(-pi/2+t_2)*sin(al_1), cos(al_1), cos(al_1)*d_2],
	               [0, 0, 0, 1]])

	T_23 = Matrix([[cos(pi/2+t_3), -sin(pi/2+t_3), 0, a_2],
	               [sin(pi/2+t_3)*cos(al_2), cos(pi/2+t_3)*cos(al_2), -sin(al_2), -sin(al_2)*d_3],
	               [sin(pi/2+t_3)*sin(al_2), cos(pi/2+t_3)*sin(al_2), cos(al_2), cos(al_2)*d_3],
	               [0, 0, 0, 1]])

	T_34 = Matrix([[cos(t_4), -sin(t_4), 0, a_3],
	               [sin(t_4)*cos(al_3), cos(t_4)*cos(al_3), -sin(al_3), -sin(al_3)*d_4],
	               [sin(t_4)*sin(al_3), cos(t_4)*sin(al_3), cos(al_3), cos(al_3)*d_4],
	               [0, 0, 0, 1]])

	T_45 = Matrix([[cos(t_5), -sin(t_5), 0, a_4],
				   [sin(t_5)*cos(al_4), cos(t_5)*cos(al_4), -sin(al_4), -sin(al_4)*d_5],
	               [sin(t_5)*sin(al_4), cos(t_5)*sin(al_4), cos(al_4), cos(al_4)*d_5],
	               [0, 0, 0, 1]])

	T_56 = Matrix([[cos(t_6), -sin(t_6), 0, a_5],
				   [sin(t_6)*cos(al_5), cos(t_6)*cos(al_5), -sin(al_5), -sin(al_5)*d_6],
	               [sin(t_6)*sin(al_5), cos(t_6)*sin(al_5), cos(al_5), cos(al_5)*d_6],
	               [0, 0, 0, 1]])

	T_56 = T_56.subs(t_6, 0)

	T0 = T_01.subs(DH_table)
	T1 = T_12.subs(DH_table)
	T2 = T_23.subs(DH_table)
	T3 = T_34.subs(DH_table)
	T4 = T_45.subs(DH_table)
	T5 = T_56.subs(DH_table)

	f1 = a_3*cos(pi/2+t_3) + d_4*sin(al_3)*sin(pi/2+t_3) + a_2
	f2 = a_3*cos(al_2)*sin(pi/2+t_3) - d_4*sin(al_3)*cos(al_2)*cos(pi/2+t_3) - d_4*sin(al_2)*cos(al_3) - d_3*sin(al_2)
	f3 = a_3*sin(al_2)*sin(pi/2+t_3) - d_4*sin(al_3)*sin(al_2)*cos(pi/2+t_3) + d_4*cos(al_2)*cos(al_3) + d_3*cos(al_2)

	DH_table = {al_0: 0, al_1: -pi/2, al_2: 0, al_3: pi/2, al_4: -pi/2, al_5: pi/2,
	            a_0: 0, a_1: 0, a_2: 22.4, a_3: 0, a_4: 0, a_5: 0,
	            d_1: 0, d_2: 0, d_3: 0, d_4: 23, d_5: 0, d_6: 0}

	# Simplifying Case 1: a_1 = 0

	r = f1**2 + f2**2 + f3**2 + d_2**2 + 2*d_2*f3 - (np.sum(T_06[:,3]**2) - T_06[3,3]**2)
	r = simplify(r.subs(DH_table))

	theta_3 = solve(r, t_3)

	# Remove angle>0 in theta_3

	temp = [degree_trans(r2d(i)) for i in theta_3]

	temp = [i for i in range(len(temp)) if temp[i] < 0]
	theta_3.pop(temp[0])

	Solution = []

	for i in range(len(theta_3)):
		Sol = [0, 0, 0, 0, 0]

		Sol[2] = theta_3[i]

		z = sin(-pi/2+t_2)*sin(al_1)*f1 + cos(-pi/2+t_2)*sin(al_1)*f2 + cos(al_1)*f3 + d_2*cos(al_1)
		z = z.subs(DH_table).subs({t_3: Sol[2]}) - T_06[2,3]

		theta_2 = solve(z, t_2)

		temp = [degree_trans(r2d(i)) for i in theta_2]
		temp = [i for i in range(len(temp)) if temp[i] < 0]
		theta_2.pop(temp[0])

		for j in range(len(theta_2)):
			Sol[1] = theta_2[j]

			x = cos(t_1)*(cos(-pi/2+t_2)*f1 - sin(-pi/2+t_2)*f2 + a_1) - sin(t_1)*(sin(-pi/2+t_2)*cos(al_1)*f1 + cos(-pi/2+t_2)*cos(al_1)*f2 - sin(al_1)*f3 - d_2*sin(al_1))
			x = x.subs(DH_table).subs({t_3: Sol[2], t_2: Sol[1]}) - T_06[0,3]

			y = sin(t_1)*(cos(-pi/2+t_2)*f1 - sin(-pi/2+t_2)*f2 + a_1) + cos(t_1)*(sin(-pi/2+t_2)*cos(al_1)*f1 + cos(-pi/2+t_2)*cos(al_1)*f2 - sin(al_1)*f3 - d_2*sin(al_1))
			y = y.subs(DH_table).subs({t_3: Sol[2], t_2: Sol[1]}) - T_06[1,3]

			theta_1 = solve(x, t_1)
			check_pi(theta_1)
			theta_1 = theta_1 + [-i for i in theta_1]
			theta_1_ = [round(i,3) for i in theta_1]
			theta_11 = solve(y, t_1)
			check_pi(theta_11)
			theta_11_ = [round(i,3) for i in theta_11]
			temp = [i for i in range(len(theta_1)) if theta_1_[i] in theta_11_]
			Sol[0] = theta_1[temp[0]]
			if temp == []:
				Sol[0] = None
			else:
				Sol[0] = theta_1[temp[0]]

			T_03 = T_01*T_12*T_23
			T_03 = T_03.subs(DH_table).subs({t_1: Sol[0], t_2: Sol[1], t_3: Sol[2]})
			T_36 = np.dot(np.linalg.inv(np.array(T_03).astype(np.float64)), T_06)

			T_36_algebra = T_34*T_45*T_56
			T_36_algebra = T_36_algebra.subs(DH_table)

			theta_5 = solve(T_36_algebra[1,2] - T_36[1,2], t_5)
			check_pi(theta_5)
			theta_5 = theta_5 + [-i for i in theta_5]
			theta_5_ = [round(i,3) for i in theta_5]
			theta_55 = solve(T_36_algebra[1,0] - T_36[1,0], t_5)
			check_pi(theta_55)
			theta_55_ = [round(i,3) for i in theta_55]
			temp = [i for i in range(len(theta_5)) if theta_5_[i] in theta_55_]
			if temp == []:
				Sol[4] = None
			else:
				Sol[4] = theta_5[temp[0]]

			theta_4 = solve(T_36_algebra[2,1] - T_36[2,1], t_4)
			check_pi(theta_4)
			theta_4 = theta_4 + [-i for i in theta_4]
			theta_4_ = [round(i,3) for i in theta_4]
			theta_44 = solve(T_36_algebra[0,1] - T_36[0,1], t_4)
			check_pi(theta_44)
			theta_44_ = [round(i,3) for i in theta_44]
			temp = [i for i in range(len(theta_4)) if theta_4_[i] in theta_44_]
			if temp == []:
				Sol[3] = None
			else:
				Sol[3] = theta_4[temp[0]]
			Solution.append(degree_trans([r2d(i) for i in Sol]))
	return Solution[0]

def angle_to_step(angle):
	"""
	Inputs: list of angles
	Outputs: list of steps
	"""
	#Motor Setup 
	Stepper1_SPR = 0.1125
	Stepper1_Tooth_ratio = 45/10

	Stepper2_SPR = 0.1125
	Stepper2_Tooth_ratio = 1

	Stepper3_SPR = 0.021875
	Stepper3_Tooth_ratio = 61/14

	Stepper4_SPR = 0.1125
	Stepper4_Tooth_ratio = 79/14

	Stepper5_SPR = 0.1125
	Stepper5_Tooth_ratio = 101/10

	Stepper = [[Stepper1_SPR, Stepper1_Tooth_ratio],
	 [Stepper2_SPR, Stepper2_Tooth_ratio],
	 [Stepper3_SPR, Stepper3_Tooth_ratio],
	 [Stepper4_SPR, Stepper4_Tooth_ratio],
	 [Stepper5_SPR, Stepper5_Tooth_ratio],]

	for i in range(len(angle)):
		angle[i] = -round(angle[i]*Stepper[i][1]/Stepper[i][0])
	return angle

def steps_for_IK(goal_state):
	"""
	Inputs: list of x-y-z offset
	Outputs: list of steps
	"""
	angle = IK(goal_state)
	angle.reverse()
	steps = angle_to_step(angle)
	return steps
