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
from utils import (connect2robot, go2home, readState, play_traj,
	get_target, send_force, joint2pose,	connect2gripper, send2gripper,
    xdot2qdot, make_traj, send_arduino, find_pos, append_data, run_xdot, pick_obj)

def drop_obj(comm_arduino, conn_gripper, pos_volt, obj):
    
    if obj == 'soft':
        send_arduino(comm_arduino, pos_volt)
        time.sleep(5)

    elif obj == 'rigid':
        send2gripper(conn_gripper, 'o')
        time.sleep(5)
    
# Define Home (Plate) Position
#PLATE = [-0.017696,  0.750638,  0.040746, -1.384955, -0.042213,  2.148325, 0.834304]
PLATE = [-0.030203,  0.526975,  0.066414, -1.793184, -0.047181,  2.350504, 0.834973]

""" 
# DEFINE PARAMETERS
parser = argparse.ArgumentParser()
parser.add_argument('--des_force', type=int, default=-25)

# Gripper Types: Modular, Granular, Soft
parser.add_argument('--gripper', type=str, default='soft ')
args = parser.parse_args()
t = time.localtime()
current_time = time.strftime("%H:%M:%S", t)
file_name = 'user study: sandwich ' + args.gripper + current_time + '.pkl'
"""

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
comm_arduino = serial.Serial('/dev/ttyACM0', baudrate=115200)
print('Connected to Arduino')

# GO TO HOME POSITION
print("Returning Home")
go2home(conn, h=PLATE)

"""

if args.gripper == 'granular ':
    pos_volt = 9.2
    neg_volt = 6
elif args.gripper == 'modular ':
    pos_volt = 1.5
    neg_volt = 9.5
elif args.gripper == 'soft ':
    pos_volt = 8.3
    neg_volt = 6
"""

step = 0
data = {"Time": [], "Position": [], "Force": [], "Voltage": []}

pos_volt = 8.3
neg_volt = 6


# Read the plate position
state = readState(conn)
plate_pos = state['x']
print(plate_pos)

#plate_pos = [0.66185158, 0.01977204, 0.15640211, -3.14121634, -0.03183889, 0.00975425]

# Trying all poses in a loop
# Define exact object positions, and type

obj_pos1 = [0.40, 0.35, 0.2, plate_pos[3], plate_pos[4], plate_pos[5]]
obj_pos2 = [0.40, 0.15, 0.2, plate_pos[3], plate_pos[4], plate_pos[5]]
obj_pos3 = [0.40, -0.05, 0.2, plate_pos[3], plate_pos[4], plate_pos[5]]
obj_pos4 = [0.40, -0.10, 0.2, plate_pos[3], plate_pos[4], plate_pos[5]]
obj_pos5 = [0.40, -0.25, 0.2, plate_pos[3], plate_pos[4], plate_pos[5]]

obj_typ1 = 'soft'
obj_typ2 = 'soft'
obj_typ3 = 'soft'
obj_typ4 = 'soft'
obj_typ5 = 'soft'

obj_poses = [obj_pos1, obj_pos2, obj_pos3, obj_pos4, obj_pos5]
obj_types = [obj_typ1, obj_typ2, obj_typ3, obj_typ4, obj_typ5]

input("[*] Enter to start test")

for i, obj_pos in enumerate(obj_poses):

    pos = obj_pos
    obj_type = obj_types[i]

    # Get start position
    state = readState(conn)
    start_pos = state['x']

    # Go to the desired position
    make_traj(start_pos, pos, 10)
    play_traj(conn, data, 'traj.pkl', pos_volt, 10)

    input("[*] Arrived at object #{} position, press enter to continue ".format(i+1))

    # Pick up the object (check if soft or rigid for trajectory)
    successful = False

    while not successful:
        pick_obj(comm_arduino, conn, conn_gripper, data, pos_volt, neg_volt, obj_type)
        
        ans = input("Was the object pick up (Y/N)?: ")
        if ans == 'Y':
            successful = True

    # Return to plate
    print("[*] Returning to plate")
    current_state = readState(conn)
    current_pos = state['x']
    make_traj(current_pos, plate_pos, 10)
    play_traj(conn, data, 'traj.pkl', pos_volt, 10) 

    # Check plate pos
    print(current_pos)

    input("[*] Press enter to drop object")
    # Drop the object
    drop_obj(comm_arduino, conn_gripper, pos_volt, obj_type)


# while True:
#     # for each object
#     for idx in range(10):
#         # go to line of food
#         make_traj(plate_pos, food_pos, 5)
#         data = play_traj(conn, data, 'traj.pkl', pos_volt, 5)
#         print('went to food')
#         # go to object
#         for jdx in range(100*idx):
#             #xdot = [0., 0.2, 0., 0., 0., 0.]
#             #run_xdot(xdot, conn)
#             #update food_pos
#             food_pos[2] += 0.2
#             make_traj(curr_pos,food_pos, 5)
#             data = play_traj(conn, data, 'traj.pkl', pos_volt, 5)
#         print('went to desired object')
#         # pick up object
#         pick_obj(comm_arduino, conn, conn_gripper, data, pos_volt, neg_volt, obj)
#         print('object picked up')
#         # bring object to plate
#         curr_pos, curr_q, states = find_pos(conn)
#         make_traj(curr_pos, plate_pos, 10)
#         data = play_traj(conn, data, 'traj.pkl', pos_volt, 10)
#         print('object on plate')
#         # drop object
#         if obj == 'rigid':
#             send2gripper(conn_gripper, "o")
#         if obj == 'soft':
#             send_arduino(comm_arduino, pos_volt)
        
#         time.sleep(10)