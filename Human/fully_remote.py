import numpy as np
import pickle
import argparse
import time
import os, sys
import serial
import os, sys
from utils import *

HOME = [-0.005758, -0.363266,  0.131343, -1.509827,  0.055365,  1.158452, 0.923992]

def main():
	# DEFINE PARAMETERS
	p = argparse.ArgumentParser()
	p.add_argument('--gripper', type=str, default='riso')
	p.add_argument('--user', type=str, required=True, help='Type in the user number to save data')
	args = p.parse_args()
	
	# define save file path
	if not os.path.exists('user_study/{}/'.format(args.gripper)):
		os.makedirs('user_study/{}/'.format(args.gripper))

	file_name = 'user_study/{}/user_{}.pkl'.format(args.gripper, args.user)

	# CONNECT TO ROBOT
	PORT_robot = 8080
	print('Connecting to low-level controller...')
	conn = connect2robot(PORT_robot)
	print("Connection Established")

	# CONNECT TO GRIPPER
	if args.gripper == 'riso':
		PORT_gripper = 8081
		print("Connecting to gripper")
		conn_gripper = connect2gripper(PORT_gripper)
		print("Connection Established")

	# CONNECT TO PRESSURE PORT AND DEFINE PRESSURE/VOLTAGE BOUNDS
	"""
	NOTE: Arduino 2 controls the pressure and Arduino 1 controls valves
	"""

	if args.gripper == 'granular':
		des_force = -35
		comm_arduino1 = serial.Serial('/dev/ttyACM2', baudrate=9600)
		comm_arduino2 = serial.Serial('/dev/ttyACM3', baudrate=19200)
		max_psi = 8 # To inflate gripper -- OPEN (+ve Pressure)
		min_psi = 0 # To grasp (Vacuum)
		max_volt = 8.4
		min_volt = 0.0
	if args.gripper == 'modular':
		des_force = -20 
		comm_arduino1 = serial.Serial('/dev/ttyACM2', baudrate=9600)
		comm_arduino2 = serial.Serial('/dev/ttyACM3', baudrate=19200)
		max_psi = 14 # to close (+ve Pressure)
		min_psi = 0 # to open (vacuum)
		max_volt = 8.4
		min_volt = 0.0
	if args.gripper == 'riso':
		des_force = -20 
		comm_arduino2 = serial.Serial('/dev/ttyACM0', baudrate=115200)
		max_volt = 8.4 # To Release
		min_volt = 0.0 # To Grasp
		max_psi = 14 
		min_psi = 0
	print('Connected to Arduino')

	# GO TO HOME POSITION
	print("Returning Home")
	go2home(conn, h=HOME)

	# SET UP FORCE WINDOW
	GUI_1 = GUI_Interface()
	GUI_1.textbox1.delete(0, END)
	GUI_1.textbox1.insert(0, round(0, 2))
	GUI_1.root.geometry("+2000+100")
	GUI_1.myLabel1 = Label(GUI_1.root, text = "Force", font=("Palatino Linotype", 50))
	GUI_1.myLabel1.grid(row = 0, column = 0, pady = 50, padx = 50)
	GUI_1.root.update()

	
	# SET UP VOLTAGE WINDOW
	GUI_2 = GUI_Interface()
	GUI_2.textbox1.delete(0, END)
	GUI_2.textbox1.insert(0, round(8.16, 2))
	GUI_2.root.geometry("+2500+100")
	if args.gripper == 'riso':
		GUI_2.myLabel1 = Label(GUI_2.root, text = "Voltage", font=("Palatino Linotype", 50))
	else:
		GUI_2.myLabel1 = Label(GUI_2.root, text = "Pressure", font=("Palatino Linotype", 50))
	GUI_2.myLabel1.grid(row = 0, column = 0, pady = 50, padx = 50)
	GUI_2.root.update()

	# INITIALIZE SAVE DATA FORMAT
	data = {"Time": [], "Position": [], "Force": [], "Inputs": [], "Voltage": []}

	# INITIALIZE VARIABLES
	action_scale = 0.3
	mode = 'v'
	interface = Joystick()
	flag_gui = False
	flag_gui2 = False
	voltage = 8.16
	pressure = 1.0
	input('[*] PRESS ENTER TO START')
	print("Ready for Teleoperation... ")


	while True:
		state = readState(conn)
		x_dot = [0]*6
		z, A_pressed, B_pressed, X_pressed, Y_pressed, START_pressed, STOP_pressed, RT, LT = interface.input()
		Joystick_inputs = [z, A_pressed, B_pressed, X_pressed, Y_pressed, START_pressed, STOP_pressed, RT, LT]

		# RIGID GRIPPER COMMANDS FOR RISO
		if A_pressed : # Close
			send2gripper(conn_gripper, "c")
			time.sleep(2.0)
			
		if B_pressed : # Open
			send2gripper(conn_gripper, "o")
			time.sleep(2.0)
			

		# PRESSURE COMMANDS FOR ALL GRIPPERS
		if LT: 
			pressure -= 1.0
			voltage -= 0.5
			pressure = np.clip(pressure, min_psi, max_psi)
			voltage = np.clip(voltage, min_volt, max_volt)
			if args.gripper == 'granular' or args.gripper == 'modular':
				if pressure == 0.0:
					send_arduino(comm_arduino2, str(pressure))
					send_arduino(comm_arduino1, '0')
					time.sleep(0.1)
				else:
					send_arduino(comm_arduino1, '1')
					send_arduino(comm_arduino2, str(pressure))
					time.sleep(0.1)
			
			if args.gripper == 'riso':
				send_arduino(comm_arduino2, str(voltage))
				time.sleep(0.1)

		if RT: 
			pressure += 1.0
			voltage += 0.5
			pressure = np.clip(pressure, min_psi, max_psi)
			voltage = np.clip(voltage, min_volt, max_volt)
			if args.gripper == 'granular' or args.gripper == 'modular':
				if pressure == 0.0:
					send_arduino(comm_arduino2, str(pressure))
					send_arduino(comm_arduino1, '0')
					time.sleep(0.1)
				else:
					send_arduino(comm_arduino1, '1')
					send_arduino(comm_arduino2, str(pressure))
					time.sleep(0.1)

			if args.gripper == 'riso':
				send_arduino(comm_arduino2, str(voltage))
				time.sleep(0.1)
			
			

		# stop the code and save recorded data	
		if START_pressed:
			pickle.dump(data, open(file_name, 'wb'))
			print("DATA SAVED")
			send_arduino(comm_arduino2, voltage)
			break

		# resets robot to home position
		if STOP_pressed:
			if args.gripper == 'riso':
				comm_arduino2 = serial.Serial('/dev/ttyACM0', baudrate=115200)
			elif args.gripper == 'modular' or args.gripper == 'granular':
				comm_arduino1 = serial.Serial('/dev/ttyACM2', baudrate=9600)
				comm_arduino2 = serial.Serial('/dev/ttyACM3', baudrate=19200)
			go2home(conn,h=HOME)

		# DEFINE WRENCH OF ROBOT
		wrench = state['O_F']

		# ROBOT CONTROLS
		x_dot[0] = action_scale * z[1]
		x_dot[1] = action_scale * z[0]
		x_dot[2] = -action_scale * z[2]
		
		if state["x"][2] <= 0.1:
			x_dot[0] = action_scale/4 * z[1]
			x_dot[1] = action_scale/4 * z[0]
			x_dot[2] = -action_scale/4 * z[2]

		# IF FORCE OBSERVED GREATER THAN 10 (default 25), DON'T MOVE IN Z
		# THIS IS IMPLEMENTED FOR ROBOT SAFETY
		if wrench[2] <= des_force:
			if x_dot[2] < 0.0:
				x_dot[2] = 0.0*x_dot[2]
		
		# CONVERT END EFFECTOR TO JOINT VELOCITY
		q_dot = xdot2qdot(x_dot, state)
		
		# UPDATE THE GUI FOR THE USERS
		flag_gui, flag_gui2 = update_gui(args.gripper, GUI_1, GUI_2, wrench, pressure, voltage, flag_gui, flag_gui2)
		

		# APPEND DATA
		data = append_data(data, time.time(), readState(conn), voltage, Joystick_inputs)

		# SEND JOINT VELOCITIES TO ROBOT
		send2robot(conn, q_dot, mode)

if __name__=='__main__':
	main()