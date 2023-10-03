from fileinput import filename
from tracemalloc import start
import numpy as np
import pickle
import matplotlib.pyplot as plt
import socket
import argparse
import cv2
import time
import serial
import os, sys
from multiprocessing import Process
from utils import connect2robot, go2home, readState, play_traj,\
	get_target, send_force, joint2pose,	connect2gripper, send2gripper,\
    xdot2qdot, make_traj, send_arduino, pick_obj, find_pos, append_data, run_xdot

# Define Start/Observe Position
HOME = [0.0, -0.0, -0.0, -1.28331, -0.0, 1.27004, 0.756409]
#HOME = [0.010475,  0.028708,  0.005619, -1.451724, -0.003384,  1.469466, 0.781665]
# HOME = [-3.482700e-02,  8.793000e-02,  1.627400e-02, -2.400776e+00, -1.049000e-03,  2.488808e+00,  7.726950e-01]
#HOME = [-0.028625,  0.027778, -0.030703, -1.461542, -0.002855,  1.478592, 0.706235]
# Define Basket Position
#BASKET = [0.444872,  0.475674,  0.235403, -1.780706, -0.182159,  2.259302, -0.007972]
#basket_pos = [0.52512205, 0.39766829, 0.19388049, 3.10404097, 0.01293037, 1.5618837]
#BASKET = [0.152112,  0.828806, -0.145881, -0.957249,  0.116908,  1.817929, 0.809513]
#basket_pos = [0.7747214 ,  0.05695364,  0.28007221, -3.13365877, -0.03823582, 0.00443894]


#LEFT = 0.47103876,  0.41723265,  0.21848192, -3.13724213, -0.05326227, 0.00482116
#RIGHT = 0.47634303, -0.32419655,  0.14712334,  3.11504136, -0.00786539, 0.04024911
# DEFINE PARAMETERS
parser = argparse.ArgumentParser()
parser.add_argument('--des_force', type=int, default=-25)

# Gripper Types: Modular, Granular, Soft
parser.add_argument('--gripper', type=str, default='modular ')
args = parser.parse_args()
t = time.localtime()
current_time = time.strftime("%H:%M:%S", t)
file_name = 'user study: autonomous ' + args.gripper + current_time + '.pkl'

# CONNECT TO ROBOT
PORT_robot = 8080
print('[*] Connecting to low-level controller...')
conn = connect2robot(PORT_robot)
print("Connection Established")

# CONNECT TO GRIPPER
PORT_gripper = 8081
print("[*] Connecting to gripper")
conn_gripper = connect2gripper(PORT_gripper)
print("Connected to gripper")

# CONNECT TO PRESSURE
comm_arduino1 = serial.Serial('/dev/ttyACM2', baudrate=9600)
print('Connected to Arduino')

comm_arduino2 = serial.Serial('/dev/ttyACM1', baudrate=19200)
print('Connected to Arduino')

step = 0
data = {"Time": [], "Position": [], "Force": [], "Inputs": [], "Voltage": []}
if args.gripper == 'granular ':
    pos_volt = '10.0'
    neg_volt = '0.0'
elif args.gripper == 'modular ':
    pos_volt = '0.0'
    neg_volt = '14.0'
elif args.gripper == 'soft ':
    pos_volt = 8.3
    neg_volt = 4

# GO TO HOME POSITION
print("Returning Home")
start_pos, start_q, states = find_pos(conn)
HOME_pos = np.array([0.5, 0., 0.5, start_pos[3], start_pos[4], start_pos[5]])
make_traj(start_pos, HOME_pos, 10)
data = play_traj(conn, data, 'traj.pkl', pos_volt, 10)

input("Enter to start test")

while True:
    P = input("Are there objects to transport? Y or N: ")
    # Pick up object
    if P == 'Y':
        # send positive voltage
        send_arduino(comm_arduino1, '0')
        send_arduino(comm_arduino2, pos_volt)
        time.sleep(2)
        # go down on top of object
        start_pos, start_q, states = find_pos(conn)
        des_pos = np.array([0.5, 0., 0.01, start_pos[3], start_pos[4], start_pos[5]])
        make_traj(start_pos, des_pos, 10)
        data = play_traj(conn, data, 'traj.pkl', pos_volt, 10)

        # send negative voltage/grasp object
        send_arduino(comm_arduino1, '1')
        send_arduino(comm_arduino2, neg_volt)
        time.sleep(2)
        # timestep = time.time()
        # while (time.time() - timestep) < 20:
        #     state = readState(conn)
        #     wrench = state['O_F']
        #     if wrench[2] > -35:
        #         xdot = [0., 0., -0.04, 0., 0., 0.]
        #     else:
        #         xdot = [0., 0., 0., 0., 0., 0.]
        #     run_xdot(xdot, conn)
        # print('object picked up')

        timestep = time.time()
        while (time.time() - timestep) < 3:
            xdot = [0., 0., 0.02, 0., 0., 0.]
            run_xdot(xdot, conn)
        
        # drop object in basket
        start_pos, start_q, states = find_pos(conn)
        basket_pos = [0.34974866, 0.59603009, 0.20601535, start_pos[3], start_pos[4], start_pos[5]]
        make_traj(start_pos, basket_pos, 10)
        data = play_traj(conn, data, 'traj.pkl', pos_volt, 10)

        send_arduino(comm_arduino1, '0')
        send_arduino(comm_arduino1, pos_volt)
        time.sleep(10)
        print('object dropped in basket')

        # go to home position
        send_arduino(comm_arduino1, '1')
        send_arduino(comm_arduino1, neg_volt)
        start_pos, start_q, states = find_pos(conn)
        make_traj(start_pos, HOME_pos, 10)
        data = play_traj(conn, data, 'traj.pkl', pos_volt, 10)
        
    if P == 'N':
        #data = append_data(data, time.time(), readState(conn), neg_volt, 0)
        print("Congrats!! You're done")
