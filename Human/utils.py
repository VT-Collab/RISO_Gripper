from operator import truediv
import numpy as np
import cv2
import time 
import pickle
import socket
import matplotlib.pyplot as plt
import pickle as pkl
import sys
from scipy.interpolate import interp1d
import pygame
import pyrealsense2 as rs
from tkinter import *
import serial

"""Home Position for Panda for all tasks"""
HOME = [0.8385, -0.0609, 0.2447, -1.5657, 0.0089, 1.5335, 1.8607]


def append_data(data, timestamp, state, voltage, Joystick_inputs):
	data["Time"].append(timestamp)
	data["Position"].append(state["x"])
	data["Force"].append(state["O_F"])
	data["Inputs"].append(Joystick_inputs)
	data["Voltage"].append(voltage)
	return data


def send_arduino(comm_arduino, user_input):
	string = '<' + str(user_input) + '>'
	comm_arduino.write(string.encode())



class Joystick(object):

	def __init__(self):
		pygame.init()
		self.gamepad = pygame.joystick.Joystick(0)
		self.gamepad.init()
		self.deadband = 0.1
		self.timeband = 0.5
		self.lastpress = time.time()

	def input(self):
		pygame.event.get()
		curr_time = time.time()
		z1 = self.gamepad.get_axis(0)
		z2 = self.gamepad.get_axis(1)
		z3 = self.gamepad.get_axis(4)
		if abs(z1) < self.deadband:
			z1 = 0.0
		if abs(z2) < self.deadband:
			z2 = 0.0
		if abs(z3) < self.deadband:
			z3 = 0.0
		A_pressed = self.gamepad.get_button(0) and (curr_time - self.lastpress > self.timeband)
		B_pressed = self.gamepad.get_button(1) and (curr_time - self.lastpress > self.timeband)
		X_pressed = self.gamepad.get_button(2) and (curr_time - self.lastpress > self.timeband)
		Y_pressed = self.gamepad.get_button(3) and (curr_time - self.lastpress > self.timeband)
		START_pressed = self.gamepad.get_button(7) and (curr_time - self.lastpress > self.timeband)
		STOP_pressed = self.gamepad.get_button(6) and (curr_time - self.lastpress > self.timeband)
		Right_trigger = self.gamepad.get_button(5)
		Left_Trigger = self.gamepad.get_button(4)
		if A_pressed or START_pressed or B_pressed:
			self.lastpress = curr_time
		return [z1, z2, z3], A_pressed, B_pressed, X_pressed, Y_pressed, START_pressed, STOP_pressed, Right_trigger, Left_Trigger


"""########## GUI design ##########"""
class GUI_Interface(object):
	def __init__(self):
		self.root = Tk()
		self.root.geometry("+100+100")
		self.root.title("Uncertainity Output")
		self.update_time = 0.02
		self.fg = '#ff0000'
		font = "Palatino Linotype"

		# X_Y Uncertainty
		self.myLabel1 = Label(self.root, text = "Force", font=(font, 40))
		self.myLabel1.grid(row = 0, column = 0, pady = 50, padx = 50)
		self.textbox1 = Entry(self.root, width = 5, bg = "white", fg=self.fg, borderwidth = 3, font=(font, 40))
		self.textbox1.grid(row = 1, column = 0,  pady = 10, padx = 20)
		self.textbox1.insert(0,0)


"""Connecting and sending pressure signals to Windows"""
def connect2pressure(PORT):
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	s.bind(('172.29.124.198', 10982))
	s.listen()
	print("listening")
	conn, addr = s.accept()
	data = str(8.16)
	conn.send(data.encode())
	return conn


"""Connecting and Sending commands to robot"""
def connect2robot(PORT):
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	s.bind(('172.16.0.3', PORT))
	s.listen()
	conn, addr = s.accept()
	return conn


def send2robot(conn, qdot, mode, traj_name=None, limit=1.0):
	if traj_name is not None:
		if traj_name[0] == 'q':
			# print("limit increased")
			limit = 1.0
	qdot = np.asarray(qdot)
	scale = np.linalg.norm(qdot)
	if scale > limit:
		qdot *= limit / scale
	send_msg = np.array2string(qdot, precision=5, separator=',', suppress_small=True)[1:-1]
	send_msg = "s," + send_msg + "," + mode + ","
	conn.send(send_msg.encode())


def connect2gripper(PORT):
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	s.bind(('172.16.0.3', PORT))

	s.listen()
	conn, addr = s.accept()
	return conn


def send2gripper(conn, arg):
	# print('-----function called')
	send_msg = arg
	conn.send(send_msg.encode())


def listen2robot(conn):
	state_length = 7 + 7 + 7 + 6 + 42
	message = str(conn.recv(2048))[2:-2]
	state_str = list(message.split(","))
	for idx in range(len(state_str)):
		if state_str[idx] == "s":
			state_str = state_str[idx + 1:idx + 1 + state_length]
			break
	try:
		state_vector = [float(item) for item in state_str]
	except ValueError:
		return None
	if len(state_vector) is not state_length:
		return None
	state_vector = np.asarray(state_vector)
	state = {}
	state["q"] = state_vector[0:7]
	state["dq"] = state_vector[7:14]
	state["tau"] = state_vector[14:21]
	state["O_F"] = state_vector[21:27]
	# print(state_vector[21:27])
	state["J"] = state_vector[27:].reshape((7, 6)).T

	# get cartesian pose
	xyz_lin, R = joint2pose(state_vector[0:7])
	beta = -np.arcsin(R[2, 0])
	alpha = np.arctan2(R[2, 1] / np.cos(beta), R[2, 2] / np.cos(beta))
	gamma = np.arctan2(R[1, 0] / np.cos(beta), R[0, 0] / np.cos(beta))
	xyz_ang = [alpha, beta, gamma]
	xyz = np.asarray(xyz_lin).tolist() + np.asarray(xyz_ang).tolist()
	state["x"] = np.array(xyz)
	return state


def readState(conn):
	while True:
		state = listen2robot(conn)
		if state is not None:
			break
	return state


def xdot2qdot(xdot, state):
	J_pinv = np.linalg.pinv(state["J"])
	return J_pinv @ np.asarray(xdot)


def joint2pose(q):
	def RotX(q):
		return np.array([[1, 0, 0, 0], [0, np.cos(q), -np.sin(q), 0], [0, np.sin(q), np.cos(q), 0], [0, 0, 0, 1]])

	def RotZ(q):
		return np.array([[np.cos(q), -np.sin(q), 0, 0], [np.sin(q), np.cos(q), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

	def TransX(q, x, y, z):
		return np.array([[1, 0, 0, x], [0, np.cos(q), -np.sin(q), y], [0, np.sin(q), np.cos(q), z], [0, 0, 0, 1]])

	def TransZ(q, x, y, z):
		return np.array([[np.cos(q), -np.sin(q), 0, x], [np.sin(q), np.cos(q), 0, y], [0, 0, 1, z], [0, 0, 0, 1]])

	H1 = TransZ(q[0], 0, 0, 0.333)
	H2 = np.dot(RotX(-np.pi / 2), RotZ(q[1]))
	H3 = np.dot(TransX(np.pi / 2, 0, -0.316, 0), RotZ(q[2]))
	H4 = np.dot(TransX(np.pi / 2, 0.0825, 0, 0), RotZ(q[3]))
	H5 = np.dot(TransX(-np.pi / 2, -0.0825, 0.384, 0), RotZ(q[4]))
	H6 = np.dot(RotX(np.pi / 2), RotZ(q[5]))
	H7 = np.dot(TransX(np.pi / 2, 0.088, 0, 0), RotZ(q[6]))
	H_panda_hand = TransZ(-np.pi / 4, 0, 0, 0.2105)
	H = np.linalg.multi_dot([H1, H2, H3, H4, H5, H6, H7, H_panda_hand])
	return H[:, 3][:3], H[:, :3][:3]


def go2home(conn, h=None):
	if h is None:
		home = np.copy(HOME)
	else:
		home = np.copy(h)
	total_time = 35.0;
	start_time = time.time()
	state = readState(conn)
	current_state = np.asarray(state["q"].tolist())

	# Determine distance between current location and home
	dist = np.linalg.norm(current_state - home)
	curr_time = time.time()
	action_time = time.time()
	elapsed_time = curr_time - start_time

	# If distance is over threshold then find traj home
	while dist > 0.02 and elapsed_time < total_time:
		current_state = np.asarray(state["q"].tolist())

		action_interval = curr_time - action_time
		if action_interval > 0.005:
			# Get human action
			qdot = home - current_state
			# qdot = np.clip(qdot, -0.3, 0.3)
			send2robot(conn, qdot, "v")
			action_time = time.time()

		state = readState(conn)
		dist = np.linalg.norm(current_state - home)
		curr_time = time.time()
		elapsed_time = curr_time - start_time

	# Send completion status
	if dist <= 0.02:
		return True
	elif elapsed_time >= total_time:
		return False


# def wrap_angles(theta):
# 	if theta < -np.pi:
# 		theta += 2*np.pi
# 	elif theta > np.pi:
# 		theta -= 2*np.pi
# 	else:
# 		theta = theta
# 	return theta


def update_gui(gripper, GUI_1, GUI_2, force, pressure, voltage, flag_1, flag_2):
	if gripper == 'granular':
		# CHOOSE COLOR TO DISPLAY END EFFECTOR FORCE
		if force[2] < -30 and not flag_1:
			GUI_1.fg = '#00ff00'
			GUI_1.textbox1 = Entry(GUI_1.root, width = 5, bg = "white", fg=GUI_1.fg, borderwidth = 3, font=("Palatino Linotype", 40))
			GUI_1.textbox1.grid(row = 1, column = 0,  pady = 10, padx = 20)
			GUI_1.textbox1.insert(0, round(force[2], 1))
			flag_1 = True

		if force[2] > -30 and flag_1:
			GUI_1.fg = '#ff0000'
			GUI_1.textbox1 = Entry(GUI_1.root, width = 5, bg = "white", fg=GUI_1.fg, borderwidth = 3, font=("Palatino Linotype", 40))
			GUI_1.textbox1.grid(row = 1, column = 0,  pady = 10, padx = 20)
			GUI_1.textbox1.insert(0, round(force[2], 1))
			flag_1 = False

		# CHOOSE COLOR TO DISPLAY PRESSURE
		if pressure < 5.9 and flag_2:
			GUI_2.fg = '#ff0000'
			GUI_2.textbox1 = Entry(GUI_2.root, width = 5, bg = "white", fg=GUI_2.fg, borderwidth = 3, font=("Palatino Linotype", 40))
			GUI_2.textbox1.grid(row = 1, column = 0,  pady = 10, padx = 20)
			GUI_2.textbox1.insert(0, round(pressure, 1))
			flag_2 = False

		if pressure  > 5.9 and not flag_2:
			GUI_2.fg = '#00ff00'
			GUI_2.textbox1 = Entry(GUI_2.root, width = 5, bg = "white", fg=GUI_2.fg, borderwidth = 3, font=("Palatino Linotype", 40))
			GUI_2.textbox1.grid(row = 1, column = 0,  pady = 10, padx = 20)
			GUI_2.textbox1.insert(0, round(pressure, 1))
			flag_2 = True

		# DISPLAY END EFFECTOR FORCE
		GUI_1.textbox1.delete(0, END)
		GUI_1.textbox1.insert(0, round(force[2], 1))
		GUI_1.root.update()

		# DISPLAY GRIPPER VOLTAGE
		GUI_2.textbox1.delete(0, END)
		GUI_2.textbox1.insert(0, round(pressure, 1))
		GUI_2.root.update()


	if gripper == 'modular':
		# CHOOSE COLOR TO DISPLAY END EFFECTOR FORCE
		if force[2] > -5 and not flag_1:
			GUI_1.fg = '#00ff00'
			GUI_1.textbox1 = Entry(GUI_1.root, width = 5, bg = "white", fg=GUI_1.fg, borderwidth = 3, font=("Palatino Linotype", 40))
			GUI_1.textbox1.grid(row = 1, column = 0,  pady = 10, padx = 20)
			GUI_1.textbox1.insert(0, round(force[2], 1))
			flag_1 = True

		if force[2] < -5 and flag_1:
			GUI_1.fg = '#ff0000'
			GUI_1.textbox1 = Entry(GUI_1.root, width = 5, bg = "white", fg=GUI_1.fg, borderwidth = 3, font=("Palatino Linotype", 40))
			GUI_1.textbox1.grid(row = 1, column = 0,  pady = 10, padx = 20)
			GUI_1.textbox1.insert(0, round(force[2], 1))
			flag_1 = False

		# CHOOSE COLOR TO DISPLAY PRESSURE
		if pressure < 7.9 and flag_2:
			GUI_2.fg = '#ff0000'
			GUI_2.textbox1 = Entry(GUI_2.root, width = 5, bg = "white", fg=GUI_2.fg, borderwidth = 3, font=("Palatino Linotype", 40))
			GUI_2.textbox1.grid(row = 1, column = 0,  pady = 10, padx = 20)
			GUI_2.textbox1.insert(0, round(pressure, 1))
			flag_2 = False

		if pressure  > 7.9 and not flag_2:
			GUI_2.fg = '#00ff00'
			GUI_2.textbox1 = Entry(GUI_2.root, width = 5, bg = "white", fg=GUI_2.fg, borderwidth = 3, font=("Palatino Linotype", 40))
			GUI_2.textbox1.grid(row = 1, column = 0,  pady = 10, padx = 20)
			GUI_2.textbox1.insert(0, round(pressure, 1))
			flag_2 = True

		# DISPLAY END EFFECTOR FORCE
		GUI_1.textbox1.delete(0, END)
		GUI_1.textbox1.insert(0, round(force[2], 1))
		GUI_1.root.update()

		# DISPLAY GRIPPER VOLTAGE
		GUI_2.textbox1.delete(0, END)
		GUI_2.textbox1.insert(0, round(pressure, 1))
		GUI_2.root.update()
		
	if gripper == 'riso':
		# CHOOSE COLOR TO DISPLAY END EFFECTOR FORCE
		if force[2] < -10 and not flag_1:
			GUI_1.fg = '#00ff00'
			GUI_1.textbox1 = Entry(GUI_1.root, width = 5, bg = "white", fg=GUI_1.fg, borderwidth = 3, font=("Palatino Linotype", 40))
			GUI_1.textbox1.grid(row = 1, column = 0,  pady = 10, padx = 20)
			GUI_1.textbox1.insert(0, round(force[2], 1))
			flag_1 = True

		if force[2] > -10 and flag_1:
			GUI_1.fg = '#ff0000'
			GUI_1.textbox1 = Entry(GUI_1.root, width = 5, bg = "white", fg=GUI_1.fg, borderwidth = 3, font=("Palatino Linotype", 40))
			GUI_1.textbox1.grid(row = 1, column = 0,  pady = 10, padx = 20)
			GUI_1.textbox1.insert(0, round(force[2], 1))
			flag_1 = False

		# CHOOSE COLOR TO DISPLAY PRESSURE
		if voltage > 4.0 and flag_2:
			GUI_2.fg = '#ff0000'
			GUI_2.textbox1 = Entry(GUI_2.root, width = 5, bg = "white", fg=GUI_2.fg, borderwidth = 3, font=("Palatino Linotype", 40))
			GUI_2.textbox1.grid(row = 1, column = 0,  pady = 10, padx = 20)
			GUI_2.textbox1.insert(0, round(voltage, 1))
			flag_2 = False

		if voltage < 4.0 and not flag_2:
			GUI_2.fg = '#00ff00'
			GUI_2.textbox1 = Entry(GUI_2.root, width = 5, bg = "white", fg=GUI_2.fg, borderwidth = 3, font=("Palatino Linotype", 40))
			GUI_2.textbox1.grid(row = 1, column = 0,  pady = 10, padx = 20)
			GUI_2.textbox1.insert(0, round(voltage, 1))
			flag_2 = True


		# DISPLAY END EFFECTOR FORCE
		GUI_1.textbox1.delete(0, END)
		GUI_1.textbox1.insert(0, round(force[2], 1))
		GUI_1.root.update()

		# DISPLAY GRIPPER VOLTAGE
		GUI_2.textbox1.delete(0, END)
		GUI_2.textbox1.insert(0, round(voltage, 1))
		GUI_2.root.update()

	return flag_1, flag_2
