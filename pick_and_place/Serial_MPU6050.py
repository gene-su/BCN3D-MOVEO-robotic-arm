'''
Get the value from MPU6050 and save it as Current_angle.npy
'''
import serial
import numpy as np

ser_Uno = serial.Serial('/dev/ttyUSB0', 115200)
ser_Uno.flushInput()

while True:
	try:
		ser_bytes = ser_Uno.readline()
		line = ser_bytes[0:len(ser_bytes)-2].decode()
		print(line)
		np.save('Current_angle', np.array([int(line)]))

	except:
		print("Keyboard Interrupt")
		break
