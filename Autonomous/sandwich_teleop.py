import socket
import time
import numpy as np
import pickle
import pygame
import sys
import argparse
import serial
from utils import send_arduino, go2home

"""
 * a minimal script for teleoperating the robot using a joystick
 * Dylan Losey, September 2020

 * To run:
 [1] in one terminal:
	navigate to ~/panda-ws/essentials
	run python3 teleop.py
 [2] in a second terminal:
	navigate to ~/libfranka/build
	run ./collab/velocity_control
"""

HOME = [3.600000e-05, -7.852010e-01,  1.350000e-04, -2.356419e+00, -3.280000e-04,  1.571073e+00,  7.856910e-01]

class Joystick(object):

    def __init__(self):
        pygame.init()

        self.gamepad = pygame.joystick.Joystick(0)
        self.gamepad.init()
        self.deadband = 0.1

    def input(self):
        pygame.event.get()
        z1 = self.gamepad.get_axis(0)
        z2 = self.gamepad.get_axis(1)
        z3 = self.gamepad.get_axis(4)
        if abs(z1) < self.deadband:
            z1 = 0.0
        if abs(z2) < self.deadband:
            z2 = 0.0
        if abs(z3) < self.deadband:
            z3 = 0.0

        A_pressed = self.gamepad.get_button(0)
        Y_pressed = self.gamepad.get_button(3)
        X_pressed = self.gamepad.get_button(2)
        START_pressed = self.gamepad.get_button(7)
        LT = self.gamepad.get_button(4)
        RT = self.gamepad.get_button(5)
        STOP_pressed = self.gamepad.get_button(6)
        return [z1, z2, z3], A_pressed, X_pressed, Y_pressed, START_pressed, LT, RT, STOP_pressed


def connect2robot(PORT):
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	s.bind(('172.16.0.3', PORT))
	s.listen()
	conn, addr = s.accept()
	return conn

def send2robot(conn, qdot, limit=1.0):
	mode = 'v'
	qdot = np.asarray(qdot)
	scale = np.linalg.norm(qdot)
	if scale > limit:
		qdot = np.asarray([qdot[i] * limit/scale for i in range(7)])
	send_msg = np.array2string(qdot, precision=5, separator=',',suppress_small=True)[1:-1]
	send_msg = "s," + send_msg + "," + mode + ","
	conn.send(send_msg.encode())

def listen2robot(conn):
	state_length = 7 + 7 + 7 + 6 + 42
	message = str(conn.recv(2048))[2:-2]
	state_str = list(message.split(","))
	for idx in range(len(state_str)):
		if state_str[idx] == "s":
			state_str = state_str[idx+1:idx+1+state_length]
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
	state["J"] = state_vector[27:].reshape((7,6)).T

	xyz_lin, R = joint2pose(state_vector[0:7])
	beta = -np.arcsin(R[2,0])
	alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
	gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
	xyz_ang = [alpha, beta, gamma]
	xyz = np.asarray(xyz_lin).tolist() + np.asarray(xyz_ang).tolist()
	state["x"] = np.array(xyz)
	return state

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
	H2 = np.dot(RotX(-np.pi/2), RotZ(q[1]))
	H3 = np.dot(TransX(np.pi/2, 0, -0.316, 0), RotZ(q[2]))
	H4 = np.dot(TransX(np.pi/2, 0.0825, 0, 0), RotZ(q[3]))
	H5 = np.dot(TransX(-np.pi/2, -0.0825, 0.384, 0), RotZ(q[4]))
	H6 = np.dot(RotX(np.pi/2), RotZ(q[5]))
	H7 = np.dot(TransX(np.pi/2, 0.088, 0, 0), RotZ(q[6]))
	H_panda_hand = TransZ(-np.pi/4, 0, 0, 0.2105)
	H = np.linalg.multi_dot([H1, H2, H3, H4, H5, H6, H7, H_panda_hand])
	return H[:,3][:3], H[:,:][:]
	
def readState(conn):
	while True:
		state = listen2robot(conn)
		if state is not None:
			break
	return state

def xdot2qdot(xdot, state):
	J_pinv = np.linalg.pinv(state["J"])
	return J_pinv @ np.asarray(xdot)
def send2gripper(conn, arg):
	# print('-----function called')
	send_msg = arg
	conn.send(send_msg.encode())

def connect2gripper(PORT):
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	s.bind(('172.16.0.3', PORT))

	s.listen()
	print("check2")
	conn, addr = s.accept()
	print("check3")
	return conn

def main(conn):

    voltage = 8.16
    max_volt = 8.4 # To release
    min_volt = 0.0 # To grasp

    #PORT_robot = 8080
    action_scale = 0.1

    #print('[*] Connecting to low-level controller...')

    #conn = connect2robot(PORT_robot)
    PORT_gripper = 8081

    print("[*] Connecting to gripper")
    conn_gripper = connect2gripper(PORT_gripper)
    print("Connected to gripper")
    grmod = 5
    interface = Joystick()

    # Connect to arduino
    comm_arduino = serial.Serial('/dev/ttyACM0', baudrate=115200)
    print('Connected to arduino')
    
    # Go to home
    print("Returning home")
    go2home(conn, h=HOME)

    print('[*] Ready for a teleoperation...')
    #print("check")

    while True:
        #print("!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        state = readState(conn)
        #print(state['q'])
        wrench = state['O_F']
        z, grasp,X_pressed,Y_pressed, stop, LT, RT, go_home = interface.input()
        if stop:
            print("[*] Done!")
            return True

        if go_home:
            go2home(conn, h=HOME)

        xdot = [0]*6
        if X_pressed:
            send2gripper(conn_gripper, "c")
            time.sleep(0.5)

        if Y_pressed:
            print("YPRESSED")
            send2gripper(conn_gripper, "o")
            time.sleep(0.5)

        if LT:
            voltage -=0.5
            voltage = np.clip(voltage, min_volt, max_volt)
            send_arduino(comm_arduino, str(voltage))
            time.sleep(0.1)

        if RT:
            voltage +=0.5
            voltage = np.clip(voltage, min_volt, max_volt)
            send_arduino(comm_arduino, str(voltage))
            time.sleep(0.1)

        if grasp:
            xdot[3] = -grmod*action_scale * z[0]
            xdot[4] = grmod*action_scale * z[1]
            xdot[5] = 2*grmod*action_scale * z[2] #fin axis
        else:
            xdot[0] = .8*action_scale * z[1]
            xdot[1] = .8*action_scale * z[0]
            xdot[2] = -2*action_scale * z[2]

        if state['x'][2] <= 0.1:
            xdot[0] = action_scale/4 * z[1]
            xdot[1] = action_scale/4 * z[0]
            xdot[2] = -action_scale/4 * z[2]

        if wrench[2] <= -10:
            if xdot[2]< 0.0:
                xdot[2] = 0.0*xdot[2]
        
        qdot = xdot2qdot(xdot, state)
        send2robot(conn, qdot)
        # time.sleep(1)


if __name__=='__main__':
	#parser = argparse.ArgumentParser()
	#parser.add_argument('--mode', type=str, default='auto')
	#parser.add_argument('--des_force', type=int, default=-25)
	#parser.add_argument('--save_name', type=str, default='test')
	#parser.add_argument('--object', type=str, required=True)

	#args = parser.parse_args()
	PORT_robot = 8080
	print('[*] Connecting to low-level controller...')
	conn = connect2robot(PORT_robot)
	print("Connection Established")
	#go2home(conn, h=HOME)

	main(conn)
