'''
Pick and place task using BCN3D MOVEO.
'''
import serial
import numpy as np
from Pieper_solution import *
import time

def adjust(goal_angle):
	'''
	adjust the camera to face down
	'''
	rotate_or_not = 0
	while True:
		#-----------Decide direction of rotation and rotate------------
		if rotate_or_not == 0:
			currnt_angle = None
			while currnt_angle is None:
				try:
					currnt_angle = np.load('Current_angle.npy')[0]
				except:
					pass
			if currnt_angle > goal_angle:
				ser.write('C21'.encode())
			else:
				ser.write('C20'.encode())
			rotate_or_not = 1
		#---------------Stop the stepper at goal_angle------------------
		currnt_angle = None
		while currnt_angle is None:
			try:
				currnt_angle = np.load('Current_angle.npy')[0]
			except:
				pass
		if currnt_angle == goal_angle:
			ser.write('Stop'.encode())
			break


class Robot_Arm(object):

	def __init__(self):
		self.current_pos = []

	def move_to(self, goal_pos, gripper):
		# gripper: 1/2, Open/Close
		if self.current_pos == []:
			self.current_pos = goal_pos
			steps_list = steps_for_IK(goal_pos)

		else:
			temp1 = steps_for_IK(goal_pos)
			temp2 = steps_for_IK(self.current_pos)

			self.current_pos = goal_pos
			steps_list = [i-j for i, j in zip(temp1,temp2)]

		print(steps_list)
		steps_string = 'C1' + str(steps_list[0]) + ',' + str(steps_list[1]) + ',' + \
							  str(steps_list[2]) + ',' + str(steps_list[3]) + ',' + \
							  str(steps_list[4]) + ',' + str(gripper)

		ser.write(steps_string.encode())

	def original_pose(self):
		steps_list = steps_for_IK(self.current_pos)
		steps_list = [-i for i in steps_list]
		steps_string = 'C1' + str(steps_list[0]) + ',' + str(steps_list[1]) + ',' + \
							  str(steps_list[2]) + ',' + str(steps_list[3]) + ',' + \
							  str(steps_list[4]) + ',0'

		ser.write(steps_string.encode())
		self.current_pos = []

def cam_offset_to(arm_x_y, cam_offset):
	'''
	turn offsets on camera into offsets which the robotic arm has to move
	'''
	dis_gripper = np.sqrt(arm_x_y[0]**2 + arm_x_y[1]**2)
	cam_x_y = [arm_x_y[0] - 2*arm_x_y[0]/dis_gripper,
		   	   arm_x_y[1] - 2*arm_x_y[1]/dis_gripper]

	cam2_x_y = [cam_x_y[0] + cam_offset[0], cam_x_y[1] + cam_offset[1]]
	dis_cam2 = np.sqrt(cam2_x_y[0]**2 + cam2_x_y[1]**2)

	arm2 = [round(cam2_x_y[0] + 2*cam2_x_y[0]/dis_cam2, 2),
			round(cam2_x_y[1] + 2*cam2_x_y[1]/dis_cam2, 2)]

	return arm2

def detecting_gesture():
	'''
	gesture to catch the whole workspace
	'''
	arm.move_to([30,0,35], 0)
	return 30, 0

def action(num):
	'''
	a series of actions to complete the task
	'''
	global step
	global x
	global y
	global ball_x
	global ball_y
	global goal_x
	global goal_y
	global offset

	if num == 1:
		x, y = detecting_gesture()
		step += 1

	elif num == 2:
		time.sleep(1)
		adjust(-15)
		step += 1
	
	elif num == 3:
		time.sleep(1)
		adjust(-8)
		step += 1	
	
	elif num == 4:
		time.sleep(1)
		offset = None
		while offset is None:
			try:
				offset = np.load('offset.npy')
			except:
				pass
		print('ball_x_y_offset:', list(offset[0:2]))
		if abs(offset[0]) < 0.5 and abs(offset[1]) < 0.5:
			arm.move_to([x, y, 11.5], 2)
			step += 1
		else:
			x_y_offset = cam_offset_to([x, y], [offset[0], offset[1]])
			x, y = x_y_offset[0], x_y_offset[1]
			arm.move_to([x, y, 35], 0)

	elif num == 5:
		x, y = detecting_gesture()
		step += 1

	elif num == 6:
		time.sleep(1)
		offset = None
		while offset is None:
			try:
				offset = np.load('offset.npy')
			except:
				pass
		print('goal_x_y_offset:', list(offset[2:4]))
		if abs(offset[2]) < 0.5 and abs(offset[3]) < 0.5:
			arm.move_to([x, y, 13], 1)
			step += 1
		else:
			x_y_offset = cam_offset_to([x, y], [offset[2], offset[3]])
			x, y = x_y_offset[0], x_y_offset[1]
			arm.move_to([x, y, 35], 0)

ser = serial.Serial('/dev/ttyUSB1', 9600)
ser.flushInput()
time.sleep(1)

arm = Robot_Arm()

step = 1
x, y = 0, 0
ball_x, ball_y, goal_x, goal_y = 0, 0, 0, 0
offset = 0

while True:
	ser_bytes = ser.readline()
	line = ser_bytes[0:len(ser_bytes)-2].decode()
	print(line)
	if line == 'OK':
		if step == 7:
			break
		action(step)

