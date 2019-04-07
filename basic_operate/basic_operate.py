'''
Function:
	move_angle() -- Make joints move specific angles
		Ex: move_angle([10,20,30,40,50]) 
	gripper() -- Open or close the gripper
		Ex: gripper(0)

Class:
	Robot_Arm()
		move_to() -- Move gripper to specific coordinate
			Ex: arm.move_to([20, 0, 30])   
									[20, 0, 30]: x-y-z offset
		original_pose() -- Move robot arm to original gesture

'''

from Pieper_solution import *

def move_angle(angle_list):
	'''
	Inputs: list of angles [10, 20, 30, 40, 50]
	'''
	if isinstance(angle_list, list) == False or len(list) != 5:
		print('Please input a list of length 5.')
	else:
		steps_list = angle_to_step(angle_list)
		steps_string = str(steps_list[0]) + ',' + str(steps_list[1]) + ',' + \
					   str(steps_list[2]) + ',' + str(steps_list[3]) + ',' + \
					   str(steps_list[4])
		ser.write(steps_string.encode())

def gripper(command):
	"""
	Inputs: 0/1 to close/open the gripper.
	"""
	if command == 1:
		ser.write('0,0,0,0,0,1'.encode())
	elif command == 0:
		ser.write('0,0,0,0,0,2'.encode())
	else:
		print('Input 0/1 to close/open the gripper.')

class Robot_Arm(object):

	def __init__(self):
		self.current_pos = []

	def move_to(self, goal_pos):

		if self.current_pos == []:
			self.current_pos = goal_pos
			steps_list = steps_for_IK(goal_pos)

		else:
			temp1 = steps_for_IK(goal_pos)
			temp2 = steps_for_IK(self.current_pos)

			self.current_pos = goal_pos
			steps_list = [i-j for i, j in zip(temp1,temp2)]

		print(steps_list)
		steps_string = str(steps_list[0]) + ',' + str(steps_list[1]) + ',' + \
					   str(steps_list[2]) + ',' + str(steps_list[3]) + ',' + \
					   str(steps_list[4])

		ser.write(steps_string.encode())

	def original_pose(self):
		steps_list = steps_for_IK(self.current_pos)
		steps_list = [-i for i in steps_list]
		steps_string = str(steps_list[0]) + ',' + str(steps_list[1]) + ',' + \
					   str(steps_list[2]) + ',' + str(steps_list[3]) + ',' + \
					   str(steps_list[4])

		ser.write(steps_string.encode())
		self.current_pos = []


import serial
ser = serial.Serial('/dev/ttyUSB1', 9600)
ser.flushInput()

'''
while True:
	ser_bytes = ser.readline()
	line = ser_bytes[0:len(ser_bytes)-2].decode()
	print(line)
'''
