import argparse
import os
import pickle
import socket
import sys
import time
from fileinput import filename
from multiprocessing import Process
from tracemalloc import start

import cv2
import matplotlib.pyplot as plt
import numpy as np
import serial

from utils import (append_data, connect2gripper, connect2robot, find_pos,
                   get_target, go2home, joint2pose, make_traj, pick_obj,
                   play_traj, readState, run_xdot, send2gripper, send_arduino,
                   send_force, xdot2qdot)

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
parser.add_argument('--gripper', type=str, default='granular ')
args = parser.parse_args()
t = time.localtime()
current_time = time.strftime("%H:%M:%S", t)
file_name = 'user study: autonomous ' + args.gripper + current_time + '.pkl'

# CONNECT TO ROBOT
PORT_robot = 8080
print('[*] Connecting to low-level controller...')
conn = connect2robot(PORT_robot)
print("Connection Established")

# depending on gripper type, comment in/out certain lines
# CONNECT TO GRIPPER
# PORT_gripper = 8081
# print("[*] Connecting to gripper")
# conn_gripper = connect2gripper(PORT_gripper)
# print("Connected to gripper")

# CONNECT TO PRESSURE
comm_arduino1 = serial.Serial('/dev/ttyACM2', baudrate=9600)
comm_arduino2 = serial.Serial('/dev/ttyACM3', baudrate=19200)
#comm_arduino2 = serial.Serial('/dev/ttyACM1', baudrate=115200)
print('Connected to Arduino')

# GO TO HOME POSITION
print("Returning Home")
go2home(conn, h=HOME)

step = 0
data = {"Time": [], "Position": [], "Force": [], "Inputs": [], "Voltage": []}
if args.gripper == 'granular ':
    pos_volt = '6.0'
    neg_volt = '0.0'
    # pos_volt = 8.35
    # neg_volt = 1
elif args.gripper == 'modular ':
    pos_volt = '0.0'
    neg_volt = '14.0'
elif args.gripper == 'soft ':
    pos_volt = 8.4
    neg_volt = 4

input("Enter to start test")
send_arduino(comm_arduino1, '1')
send_arduino(comm_arduino2, pos_volt)
time.sleep(15)
while True:
    #P = input("Are there objects to transport? Y or N: ")
    P = 'Y'
    # Pick up object
    if P == 'Y':
        # Find start position
        start_pos, start_q, states = find_pos(conn)
        # Detect the desired object
        y, x, z, obj = get_target()
        print(x, y, z, obj)
        des_pos = np.array([x, y, z+0.2, start_pos[3], start_pos[4], start_pos[5]])
        if z == 0.755 or z == 0.735:
            des_pos = np.array([x, y, 0.2, start_pos[3], start_pos[4], start_pos[5]])

        # go to desired object
        make_traj(start_pos, des_pos, 10)
        data = play_traj(conn, data, 'traj.pkl', pos_volt, 10)
        print('traveled to desired object')
        start_pos = des_pos
        #send2gripper(conn_gripper, "o")

        #pick up desired object
        # if obj == 'rigid': 
        #     des_pos = np.array([x, y, z-0.8*z, start_pos[3], start_pos[4], start_pos[5]])
        #     if z == 0.755 or z == 0.735:
        #         des_pos = np.array([x, y, 0., start_pos[3], start_pos[4], start_pos[5]])
        #     make_traj(start_pos, des_pos, 10)
        #     data = play_traj(conn, data, 'traj.pkl', pos_volt, 10)
        #     send2gripper(conn_gripper, "c")
        #     time.sleep(2)
        if obj == 'rigid':
            #send_arduino(comm_arduino1, '1')
            # send_arduino(comm_arduino2, pos_volt)
            # time.sleep(10)
            des_pos = np.array([x, y, 0.0, start_pos[3], start_pos[4], start_pos[5]])
            if z == 0.755 or z == 0.735:
                des_pos = np.array([x, y, 0.00 , start_pos[3], start_pos[4], start_pos[5]])
            make_traj(start_pos, des_pos, 10)
            data = play_traj(conn, data, 'traj.pkl', pos_volt, 10)
            # send_arduino(comm_arduino, 7.0)
            send_arduino(comm_arduino1, '0')
            send_arduino(comm_arduino2, neg_volt)
            timestep = time.time()
            while (time.time() - timestep) < 30:
                state = readState(conn)
                wrench = state['O_F']
                if wrench[2] > -25:
                    xdot = [0., 0., -0.01, 0., 0., 0.]
                else:
                    xdot = [0., 0., 0., 0., 0., 0.]
                run_xdot(xdot, conn)    
        
        print('object picked up')

        timestep = time.time()
        while (time.time() - timestep) < 10:
            xdot = [0., 0., 0.02, 0., 0., 0.]
            run_xdot(xdot, conn)
        
        # drop object in basket
        start_pos, start_q, states = find_pos(conn)
        basket_pos = [0.45, -0.65, 0.25, start_pos[3], start_pos[4], start_pos[5]]
        make_traj(start_pos, basket_pos, 10)
        data = play_traj(conn, data, 'traj.pkl', pos_volt, 10)

        # if obj == 'rigid':
        #     send2gripper(conn_gripper, "o")
        #     time.sleep(2)
        if obj == 'rigid':
            send_arduino(comm_arduino1, '1')
            send_arduino(comm_arduino2, pos_volt)
            time.sleep(15)
        print('object dropped in basket')

        # go to home position
        #send_arduino(comm_arduino1, '0')
        #send_arduino(comm_arduino2, neg_volt)
        go2home(conn, h=HOME)
        send_arduino(comm_arduino1, '1')
        send_arduino(comm_arduino2, pos_volt)
        
    if P == 'N':
        print("Congrats!! You're done")
