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

def find_pos(conn):
    state = readState(conn)
    return state['x'], state['O_F']


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


def run_xdot(xdot, conn):
    state = readState(conn)
    qdot = xdot2qdot(xdot, state)
    send2robot(conn, qdot, 'v')


def plot_data(force_arr, dist_arr):
	fig, (ax1, ax2) = plt.subplots(2)
	ax1.plot(force_arr)
	ax2.plot(dist_arr)
	plt.show()

def make_traj(start_pos, des_pos, val):
	traj = []
	for idx in range(val):
		traj.append(start_pos + (1/val) * idx * (des_pos - start_pos))
	traj.append(des_pos)
	pickle.dump(traj, open('traj.pkl', 'wb'))

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


class Trajectory(object):

	def __init__(self, xi, T):
		""" create cublic interpolators between waypoints """
		self.xi = np.asarray(xi)
		self.T = T
		self.n_waypoints = xi.shape[0]
		timesteps = np.linspace(0, self.T, self.n_waypoints)
		self.f1 = interp1d(timesteps, self.xi[:,0], kind='cubic')
		self.f2 = interp1d(timesteps, self.xi[:,1], kind='cubic')
		self.f3 = interp1d(timesteps, self.xi[:,2], kind='cubic')
		self.f4 = interp1d(timesteps, self.xi[:,3], kind='cubic')
		self.f5 = interp1d(timesteps, self.xi[:,4], kind='cubic')
		self.f6 = interp1d(timesteps, self.xi[:,5], kind='cubic')
		# self.f7 = interp1d(timesteps, self.xi[:,6], kind='cubic')

	def get(self, t):
		""" get interpolated position """
		if t < 0:
			q = [self.f1(0), self.f2(0), self.f3(0), self.f4(0), self.f5(0), self.f6(0)]
		elif t < self.T:
			q = [self.f1(t), self.f2(t), self.f3(t), self.f4(t), self.f5(t), self.f6(t)]
		else:
			q = [self.f1(self.T), self.f2(self.T), self.f3(self.T), self.f4(self.T), self.f5(self.T), self.f6(self.T)]
		return np.asarray(q)


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

"""Obtain the location of target"""
def get_target():
	#x and y adjustment for later use
	#Right EE = -.34, .71
	#Center -.261 .71
	#Left	-.180 .71
	# Configure depth and color streams
	pipeline = rs.pipeline()
	config = rs.config()
	# Get device product line for setting a supporting resolution
	pipeline_wrapper = rs.pipeline_wrapper(pipeline)
	pipeline_profile = config.resolve(pipeline_wrapper)
	device = pipeline_profile.get_device()
	device_product_line = str(device.get_info(rs.camera_info.product_line))

	found_rgb = False
	for s in device.sensors:
		if s.get_info(rs.camera_info.name) == 'RGB Camera':
			found_rgb = True
			break
	if not found_rgb:
		print("The demo requires Depth camera with Color sensor")
		exit(0)

	# device.hardware_reset()

	config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
	config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

	# Start streaming
	profile = pipeline.start(config)

	# Getting the depth sensor's depth scale (see rs-align example for explanation)
	depth_sensor = profile.get_device().first_depth_sensor()
	depth_scale = depth_sensor.get_depth_scale()
	# print("Depth Scale is: " , depth_scale)

	# We will be removing the background of objects more than
	#  clipping_distance_in_meters meters away
	clipping_distance_in_meters = 1 #1 meter
	clipping_distance = clipping_distance_in_meters / depth_scale

	# Create an align object
	# rs.align allows us to perform alignment of depth frames to others frames
	# The "align_to" is the stream type to which we plan to align depth frames.
	align_to = rs.stream.color
	align = rs.align(align_to)

	try:
		# Wait for a coherent pair of frames: depth and color
		frames = pipeline.wait_for_frames()
		aligned_frames = align.process(frames)
		depth_frame = aligned_frames.get_depth_frame()
		color_frame = aligned_frames.get_color_frame()
		# Convert images to numpy arrays
		depth_image = np.asanyarray(depth_frame.get_data())
		depth_image = depth_image[170:480,0:640]
		color_image = np.asanyarray(color_frame.get_data())
		# Convert image to black and white
		gray_img = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
		gray_img = gray_img[170:480,0:640]
		gray_upper = 110 #110
		gray_lower = 0

		kernal = np.ones ((2, 2), "uint8")

		gray1 = cv2.inRange(gray_img, gray_lower, gray_upper)
		gray1 = cv2.morphologyEx(gray1, cv2.MORPH_OPEN, kernal)

		# FOR CAMERA 1
		(contoursred1, hierarchy) =cv2.findContours (gray1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		for pic, contourred in enumerate (contoursred1):
			area = cv2.contourArea (contourred) 
			if (area > 0):
				x, y, w, h = cv2.boundingRect (contourred)
				gray_img = cv2.rectangle (gray_img, (x, y), (x + w, y + h), (0, 0, 255), 2)
				cv2.putText(gray_img,"OBJECT",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255))

		if len(contoursred1) > 0:
			# Find the biggest contour
			biggest_contour = max(contoursred1, key=cv2.contourArea)
			#print(biggest_contour)
			# Find center of contour and draw filled circle
			moments = cv2.moments(biggest_contour)
			centre_of_contour = (int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00']))
			cv2.circle(gray_img, centre_of_contour, 2, (0, 0, 255), -1)
			# Save the center of contour so we draw line tracking it
			center_points1 = centre_of_contour
			r1 = center_points1[0]
			c1 = center_points1[1]
			"""
			Take the depth as (y,x) when calling it from the image_depth matrix
			The x and y axis are flipped
			"""
			#print(r1, c1)
			x = r1 - 580
			y = 150 - c1
			#print(x, y)
			# print("The depth at [{},{}] is {}".format(x_adjust-(x/(385*0.435)),y_adjust+(y/(175*0.2)),depth_image[c1, r1]))

		# if len(contoursred1) > 0:
		# 	centers = []
		# 	heights = []
		# 	# Find the biggest contour
		# 	# biggest_contour = max(contoursred1, key=cv2.contourArea)
		# 	# print(biggest_contour)
		# 	for contour in contoursred1:
		# 		if cv2.contourArea(contour) > 5:
		# 		# Find center of contour and draw filled circle
		# 			moments = cv2.moments(contour)
		# 			centre_of_contour = (int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00']))
		# 			cv2.circle(gray_img, centre_of_contour, 2, (0, 0, 255), -1)
		# 			# Save the center of contour so we draw line tracking it
		# 			center_points1 = centre_of_contour
		# 			r1 = center_points1[0]
		# 			c1 = center_points1[1]
		# 			"""
		# 			Take the depth as (y,x) when calling it from the image_depth matrix
		# 			The x and y axis are flipped
		# 			"""
		# 			centers.append(np.array([r1, c1]))
		# 			heights.append(depth_image[c1, r1])
		# 			# print(r1, c1)
		# 			# x = r1 - 580
		# 			# y = 150 - c1
		# 			# print(x, y)

		# 	pick_obj = centers[np.argmin(heights)]
		# 	x = pick_obj[0] - 580
		# 	y = 150 - pick_obj[1]

		# Apply colormap on depth image (image must be converted to 8-bit per pixel first)
		depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

		depth_colormap_dim = depth_image.shape
		color_colormap_dim =     color_image.shape

		# If depth and color resolutions are different, resize color image to match depth image for display
		if depth_colormap_dim != color_colormap_dim:
			resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
			image_rgb = resized_color_image
			image_depth = depth_colormap
	
		else:
			image_rgb = color_image
			image_depth = depth_colormap

		# take start position 
		#cv2.imshow('RealSense RGB', gray_img)
		# cv2.imshow('RealSense Depth', image_depth)
		#cv2.waitKey(0)
		# print(x,y)
		#print('depth image', np.min(heights))
		# print(x, y, c1, r1)
		print('x ', 0.72-0.2525 + (y*0.0011), 'y ', -0.26 + (x*-0.00115), 'depth_image ',  depth_image[c1, r1])

		#type_grip = input("S: soft, R: rigid: ")
		type_grip = 'R'
		#if np.min(heights) > 600:
		if type_grip == 'S':
			x_adjust = -.32
			y_adjust = .72-0.39
			return x_adjust + (x*-0.00115), y_adjust + (y*0.0011), 0.69 - depth_image[c1, r1]/1000 + 0.045, 'soft'
			return x_adjust -(x*.00111), y_adjust +(y*.001095), 0.69-depth_image[c1, r1]/1000+0.045, 'soft' #'soft'
			#return x_adjust -(x/395*0.435), y_adjust +(y/172.5*0.2), 0.69-depth_image[c1, r1]/1000+0.035, 'soft'  #Old defaults are .332 and .74

		else:
			# goes outtooo far, reduce value of slope
			print("!!!!!!!!!")
			#x_adjust = -.1925
			x_adjust = -0.26 #-0.22 - 0.042
			# y_adjust =  .72
			y_adjust = 0.72 - 0.4
			return x_adjust + (x*-0.00114), y_adjust + (y*0.0011), 0.8 - depth_image[c1, r1]/950 - 0.045, 'rigid'
			#return x_adjust -(x*.00095), y_adjust +(y*.0011), 0.80-depth_image[c1, r1]/950 - 0.045, 'rigid'
		# if cv2.waitKey(1) & 0xFF == ord('q'):
		#     break

	finally:

		# Stop streaming
		pipeline.stop()

def wrap_angles(theta):
	if theta < -np.pi:
		theta += 2*np.pi
	elif theta > np.pi:
		theta -= 2*np.pi
	else:
		theta = theta
	return theta

def play_traj(conn, data, traj_name, voltage, total_time):
	traj = np.array(pickle.load(open(traj_name, "rb")))
	traj = Trajectory(traj[:, :6], total_time)
	start_t = time.time()
	interface = Joystick()
	while True:
		z, A_pressed, B_pressed, X_pressed, Y_pressed, START_pressed, STOP_pressed, RT, LT = interface.input()
		curr_t = time.time() - start_t
		state = readState(conn)
		x_des = traj.get(curr_t)
		x_curr = state['x']
		wrench = state['O_F']
		x_des[3] = wrap_angles(x_des[3])
		x_des[4] = wrap_angles(x_des[4])
		x_des[5] = wrap_angles(x_des[5])
		xdot = 1 * (x_des - x_curr)
		xdot[3] = wrap_angles(xdot[3])
		xdot[4] = wrap_angles(xdot[4])
		xdot[5] = wrap_angles(xdot[5])

		qdot = xdot2qdot(xdot, state)
		send2robot(conn, qdot, 'v', traj_name)
		if curr_t > total_time or wrench[2] < -20 or A_pressed:
			return data
		#data = append_data(data, time.time(), state, voltage, 0)

# go fast until distance reached, then go slow?
def pick_obj(comm_arduino, conn, conn_gripper, data, pos_volt, neg_volt, obj):
	down = True
	pick = False
	up = False
	if obj == 'soft':
		send_arduino(comm_arduino, pos_volt)
		time.sleep(10)
		while True:
			if down:
				#xdot = [0., 0., -0.1, 0., 0., 0.]
				xdot = [0., 0., -0.05, 0., 0., 0.]
				run_xdot(xdot, conn)
				state = readState(conn)
				wrench = state['O_F']
				#print(wrench[2])
				#data = append_data(data, time.time(), state, pos_volt, 0)
				if wrench[2] < -5: # originally 3
					
					pick = True
					down = False
					send_arduino(comm_arduino, neg_volt)
			
			if pick:
				#xdot = [0., 0., -0.05, 0., 0., 0.]
				xdot = [0., 0., -0.05, 0., 0., 0.]
				run_xdot(xdot, conn)
				state = readState(conn)
				wrench = state['O_F']
				#print(wrench[2])
				#data = append_data(data, time.time(), state, neg_volt, 0)
				if wrench[2] < -20: # originally -10
					up = True
					pick = False
					#up_time = time.time()
					#time.sleep(2)
					time.sleep(10)
					up_time = time.time()

			if up:
				#xdot = [0., 0., 0.5, 0., 0., 0.]
				xdot = [0., 0., 0.05, 0., 0., 0.]
				run_xdot(xdot, conn)
				state = readState(conn)
				#data = append_data(data, time.time(), state, neg_volt, 0)
				if (time.time() - up_time) > 2: #originally 2
					return data
		
	elif obj == 'rigid':
		send2gripper(conn_gripper, "o")
		# go down until above object
		timestep = time.time()
		while (time.time()-timestep) < 5.5:
			xdot = [0., 0., -0.1, 0., 0., 0.]
			run_xdot(xdot, conn)
			state = readState(conn)
			#data = append_data(data, time.time(), state, neg_volt, 0)
		# close gripper around object
		send2gripper(conn_gripper, "c")
		time.sleep(2)
		timestep = time.time()
		while (time.time() -timestep) < 2:
			xdot = [0., 0., 0.5, 0., 0., 0.]
			run_xdot(xdot, conn)
			state = readState(conn)
			#data = append_data(data, time.time(), state, neg_volt, 0)

	return data
	
def shared_pick(comm_arduino, conn, conn_gripper, data, pos_volt, neg_volt, obj, action_scale = 0.3):
	interface = Joystick()
	down = True
	pick = False
	up = False
	if obj == 'soft':
		send_arduino(comm_arduino, pos_volt)
		time.sleep(5)
		while True:
			z, A_pressed, B_pressed, X_pressed, Y_pressed, START_pressed, STOP_pressed, RT, LT = interface.input()
			Joystick_inputs = [z, A_pressed, B_pressed, X_pressed, Y_pressed, START_pressed, STOP_pressed, RT, LT]
			if down:
				xdot = [0., 0., -0.1, 0., 0., 0.]
				xdot[0] = action_scale * z[1]
				xdot[1] = action_scale * z[0]
				run_xdot(xdot, conn)
				state = readState(conn)
				wrench = state['O_F']
				print(wrench[2])
				data = append_data(data, time.time(), state, pos_volt, Joystick_inputs)
				if wrench[2] < -3:
					pick = True
					down = False
					send_arduino(comm_arduino, neg_volt)
			
			if pick:
				xdot = [0., 0., -0.05, 0., 0., 0.]
				xdot[0] = action_scale * z[1]
				xdot[1] = action_scale * z[0]
				run_xdot(xdot, conn)
				state = readState(conn)
				wrench = state['O_F']
				print(wrench[2])
				data = append_data(data, time.time(), state, neg_volt, Joystick_inputs)
				if wrench[2] < -10:
					down = True
					pick = False
					up_time = time.time()
					time.sleep(2)

			if up:
				xdot = [0., 0., 0.5, 0., 0., 0.]
				xdot[0] = action_scale * z[1]
				xdot[1] = action_scale * z[0]
				run_xdot(xdot, conn)
				state = readState(conn)
				if (time.time() - up_time) > 2:
					return data
		
	elif obj == 'rigid':
		z, A_pressed, B_pressed, X_pressed, Y_pressed, START_pressed, STOP_pressed, RT, LT = interface.input()
		send2gripper(conn_gripper, "o")
		# go down until above object
		while distance() > 0.001:
			xdot = [0., 0., -0.1, 0., 0., 0.]
			xdot[0] = action_scale * z[1]
			xdot[1] = action_scale * z[0]
			run_xdot(xdot, conn)
		# close gripper around object
		send2gripper(conn_gripper, "c")
		time.sleep(2)
	return data

def send_force(conn, conn2, data, args):
	des_force = args.des_force
	action_scale = 0.0005
	interface = Joystick()
	run_controller = False
	mode = 'v'

	state = readState(conn)
	Baseline1 = state['O_F']
	Baseline2 = Baseline1[2]
	
	BaseUse = Baseline1
	Base_prior = BaseUse[2]
	contact_switch = False
	run_controller = False
	switched = False
	
	flag = False
	voltage = data["Voltage"][-1]
	# voltage = 1.0

	while True:
		timestamp = time.time()
		state = readState(conn)
		z, A_pressed, B_pressed, X_pressed, Y_pressed, START_pressed, STOP_pressed, RT, LT = interface.input()

		Joystick_inputs = [z, A_pressed, B_pressed, X_pressed, Y_pressed, START_pressed, STOP_pressed, RT, LT]

		if not flag:
			send_arduino(conn2, voltage)
			run_controller = True
			start_time = time.time()
			if (contact_switch == False):
				Baseline2 = -2
			des_force = des_force + Baseline2
			start_t = time.time()
			time_elapsed = 0.0
			flag = True
		

		wrench = state['O_F']
		x_dot = 6*[0]

		if not contact_switch  and not switched:
			difference = BaseUse - wrench
			if difference[2] > .625 and wrench[2] < -2.5: 
				contact_switch = True
				switched = True
			else:
				BaseUse = wrench
				Base_prior = BaseUse[2]

		if contact_switch and switched:
			print("Switched")
			print(Base_prior)
			time.sleep(.5)
			des_force = des_force + (Base_prior)
			switched = False
			

		if run_controller:
			z_force_error = des_force - wrench[2]
			x_dot[2] = action_scale * z_force_error
			time_elapsed = time.time() - start_t
			if START_pressed or time_elapsed>10.0:
				print("[*] Done")
				run_controller = False
				return data
			print(wrench, des_force)

		q_dot = xdot2qdot(x_dot, state)

		data["Time"].append(timestamp)
		data["Position"].append(state["x"])
		data["Force"].append(state["O_F"])
		data["Inputs"].append(Joystick_inputs)
		data["Voltage"].append(voltage)

		
		send2robot(conn, q_dot, mode)
	


def teleop(conn, conn_gripper, args):

	GUI_1 = GUI_Interface()
	GUI_1.textbox1.delete(0, END)
	GUI_1.textbox1.insert(0, round(0, 2))
	GUI_1.root.geometry("+500+100")
	GUI_1.myLabel1 = Label(GUI_1.root, text = "Force", font=("Palatino Linotype", 40))
	GUI_1.myLabel1.grid(row = 0, column = 0, pady = 50, padx = 50)
	GUI_1.root.update()

	des_force = args.des_force
	action_scale = 0.5
	force_scale = 0.001
	mode = 'v'
	interface = Joystick()

	fast_flag = True

	print("[*] Ready for Teleoperation... ")

	state = readState(conn)
	Baseline1 = state['O_F']
	Baseline2 = Baseline1[2]
	
	BaseUse = Baseline1
	Base_prior = BaseUse[2]
	contact_switch = False
	run_controller = False
	switched = False
	voltage = 8.16
	flag_gui = False

	while True:
		timestamp = time.time()
		state = readState(conn)
		z, A_pressed, B_pressed, X_pressed, Y_pressed, START_pressed, STOP_pressed, RT, LT = interface.input()

		Joystick_inputs = [z, A_pressed, B_pressed, X_pressed, Y_pressed, START_pressed, STOP_pressed, RT, LT]


		# if START_pressed:
		# 	print("[*] Done")
		# 	return data

		if A_pressed:
			fast_flag = False
			if (contact_switch == False):
				Baseline2 = -2
			des_force = des_force + Baseline2
		if B_pressed:
			fast_flag = True

		if X_pressed:
			send2gripper(conn_gripper, "c")
			time.sleep(0.5)

		if Y_pressed:
			print("YPRESSED")
			send2gripper(conn_gripper, "o")
			time.sleep(0.5)

		if RT:
			voltage += 0.5
			if voltage >= 8.4:
				voltage = 8.4
			time.sleep(0.2)

		if LT:
			voltage -= 0.5
			if voltage <= 0.0:
				voltage = 0.0
			time.sleep(0.2)

		wrench = state['O_F']
		x_dot = [0]*6

		if not contact_switch  and not switched:
			difference = BaseUse - wrench
			if difference[2] > .625 and wrench[2] < -2.5: 
				contact_switch = True
				switched = True
			else:
				BaseUse = wrench
				Base_prior = BaseUse[2]

		if contact_switch and switched:
			print("Switched")
			print(Base_prior)
			time.sleep(.5)
			des_force = des_force + (Base_prior)
			switched = False

		if fast_flag:
			x_dot[0] = action_scale * z[1]
			x_dot[1] = action_scale * z[0]
			if state["x"][2] < 0.1:
				x_dot[2] = -action_scale/5 * z[2]
			else:
				x_dot[2] = -action_scale * z[2]

		if not fast_flag:
			x_dot[0] = -force_scale * z[1]
			x_dot[1] = -force_scale * z[0]
			x_dot[2] = -force_scale * z[2]
			


		q_dot = xdot2qdot(x_dot, state)
		# print(wrench[2])
		if wrench[2]<-28 and not flag_gui:
			GUI_1.fg = '#00ff00'
			GUI_1.textbox1 = Entry(GUI_1.root, width = 5, bg = "white", fg=GUI_1.fg, borderwidth = 3, font=("Palatino Linotype", 40))
			GUI_1.textbox1.grid(row = 1, column = 0,  pady = 10, padx = 20)
			GUI_1.textbox1.insert(0, round(wrench[2], 1))
			flag_gui = True

		if wrench[2]>-28 and flag_gui:
			GUI_1.fg = '#ff0000'
			GUI_1.textbox1 = Entry(GUI_1.root, width = 5, bg = "white", fg=GUI_1.fg, borderwidth = 3, font=("Palatino Linotype", 40))
			GUI_1.textbox1.grid(row = 1, column = 0,  pady = 10, padx = 20)
			GUI_1.textbox1.insert(0, round(wrench[2], 1))
			flag_gui = False

		GUI_1.textbox1.delete(0, END)
		GUI_1.textbox1.insert(0, round(wrench[2], 1))
		GUI_1.root.update()

		if wrench[2] <= des_force:
			if x_dot[2] < 0:
				print("Returning Zero Velocity")
				q_dot = 0*q_dot

		# data["Time"].append(timestamp)
		# data["Position"].append(state["x"])
		# data["Force"].append(state["O_F"])
		# data["Inputs"].append(Joystick_inputs)
		# data["Voltage"].append(voltage)

		

		send2robot(conn, q_dot, mode)



def pressure_control():
	interface = Joystick()
	GUI = GUI_Interface()
	GUI.textbox1.delete(0, END)
	GUI.textbox1.insert(0, round(8.16, 2))
	GUI.root.geometry("+100+100")
	GUI.myLabel1 = Label(GUI.root, text = "Voltage", font=("Palatino Linotype", 40))
	GUI.myLabel1.grid(row = 0, column = 0, pady = 50, padx = 50)
	GUI.root.update()

	voltage = 8.16
	flag_gui = False

	comm_arduino = serial.Serial('/dev/ttyACM0', baudrate=115200)



	dist_arr= []
	while True:
		# dist = comm_arduino.readline()
		# dist_arr.append(str(dist))
		z, A_pressed, B_pressed, X_pressed, Y_pressed, START_pressed, STOP_pressed, RT, LT = interface.input()
		if RT:
			voltage += 0.5
			if voltage >= 8.4:
				voltage = 8.4
			data = str(voltage)
			if voltage>=4.9 and flag_gui:
				GUI.fg = '#ff0000'
				GUI.textbox1 = Entry(GUI.root, width = 5, bg = "white", fg=GUI.fg, borderwidth = 3, font=("Palatino Linotype", 40))
				GUI.textbox1.grid(row = 1, column = 0,  pady = 10, padx = 20)
				GUI.textbox1.insert(0, round(voltage, 1))
				flag_gui = False
			GUI.textbox1.delete(0, END)
			GUI.textbox1.insert(0, round(voltage, 1))
			GUI.root.update()
			# conn.send(data.encode())
			send_arduino(comm_arduino, voltage)
			time.sleep(0.2)

		if LT:
			voltage -= 0.5
			if voltage <= 0.0:
				voltage = 0.0
			data = str(voltage)
			if voltage<4.9 and not flag_gui:
				GUI.fg = '#00ff00'
				GUI.textbox1 = Entry(GUI.root, width = 5, bg = "white", fg=GUI.fg, borderwidth = 3, font=("Palatino Linotype", 40))
				GUI.textbox1.grid(row = 1, column = 0,  pady = 10, padx = 20)
				GUI.textbox1.insert(0, round(voltage, 1))
				flag_gui = True
				
			GUI.textbox1.delete(0, END)
			GUI.textbox1.insert(0, round(voltage, 1))
			GUI.root.update()
			# conn.send(data.encode())
			send_arduino(comm_arduino, voltage)
			time.sleep(0.2)

		if STOP_pressed or exit == 'e':
			data = str(8.16)
			# conn.send(data.encode())
			send_arduino(comm_arduino, data)
			time.sleep(1)
			data_arr = []
			for idx in range(len(dist_arr)):
				temp = dist_arr[idx][2:]
				data_arr.append(temp[:-5])
			file = open('data.txt', mode='w')
			for idx in range(len(data_arr)-1):
				if len(data_arr[idx+1])== 0:
					continue
				file.write(data_arr[idx+1] + '\n')
			file.close()
			# data = 's'
			# conn.send(data.encode())
			# send_arduino(comm_arduino, data)
			print("[*] Done")
			print(dist_arr)
			return False

