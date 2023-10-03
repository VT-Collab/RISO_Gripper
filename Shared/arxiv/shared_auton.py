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
                   get_target, go2home, joint2pose, make_traj,
                   readState, run_xdot, send2gripper, send_arduino,
                   xdot2qdot, Joystick, play_shared_traj)

# Define Start Position
HOME = [3.600000e-05, -7.852010e-01,  1.350000e-04, -2.356419e+00, -3.280000e-04,  1.571073e+00,  7.856910e-01]

# Define Parameters
p = argparse.ArgumentParser()
p.add_argument('--des_force', type=int, default=-15)
p.add_argument('--pos_volt', type=int, default=8.35)
p.add_argument('--neg_volt', type=int, default=4.0)
# p.add_argument('--user', type=str, required=True, help='Type in the user number to save data')
args = p.parse_args()
interface = Joystick()
data = {"Time": [], "Position": [], "Force": [], "Inputs": [], "Voltage": []}

# Define save file path
# file_name = 'shared_study/user_{}.pkl'.format(args.user)

# CONNECT TO ROBOT
PORT_robot = 8080
print('Connecting to low-level controller...')
conn = connect2robot(PORT_robot)
print("Connection Established")

# CONNECT TO GRIPPER
PORT_gripper = 8081
print("Connecting to gripper")
conn_gripper = connect2gripper(PORT_gripper)
print("Connection Established")

# CONNECT TO PRESSURE PORT
comm_arduino = serial.Serial('/dev/ttyACM0', baudrate=115200)
print('Connected to Arduino')

# GO TO HOME POSITION
print("Returning Home")
go2home(conn, h=HOME)

# Initialize Variables
choose = True
bayesian = False
shared = False
drop = False
# practice = True
v = args.neg_volt
# send_arduino(comm_arduino, v)
input('Enter to Start Trial')
while True:
    z, A_pressed, B_pressed, X_pressed, Y_pressed, START_pressed, STOP_pressed, RT, LT = interface.input()
    Joystick_inputs = [z, A_pressed, B_pressed, X_pressed, Y_pressed, START_pressed, STOP_pressed, RT, LT]
    cur_pos, wrench = find_pos(conn)
    x_dot = [0.] * 6

    # if practice:
    #     des_pos = [0.3, 0., 0.3, cur_pos[3], cur_pos[4], cur_pos[5]]
    #     make_traj(cur_pos, des_pos, 10)
    #     data = play_shared_traj(conn, data, 'traj.pkl', v, 10)
    #     x, y = get_target(obj, cur_pos)

    # USER: Pause to allow user to pick using soft/rigid component of gripper
    if choose:
        if A_pressed: obj = 'soft'; v = args.pos_volt; send_arduino(comm_arduino, v)
        if B_pressed: obj = 'rigid'

        if A_pressed or B_pressed: 
            choose = False
            bayesian = True
            go2home(conn, h=HOME)
            print('gripper type picked, choose object and go towards it')

    # ROBOT: Go down (z plane), Bayesian Inference to determine desired object
    # USER: Control x-y plane, pick desired object
    if bayesian:
        # User Control
        x_dot[0] = z[1]
        x_dot[1] = z[0]
        # Robot Control
        x_dot[2] = -0.1

        if cur_pos[2] < 0.3:
            #x, y = get_target(obj, cur_pos)
            x = 0.4
            y = 0.
            bayesian = False
            shared = True
            first_loop = True
            print('desired object detected, picking up')

    # Once bayesian gives desired object, transfer to Shared Autonomy Mode to pick up object
    if shared:

        # Determine trajectory and pick up object/allow human to have some control
        if first_loop:
            des_pos = [x, y, 0., cur_pos[3], cur_pos[4], cur_pos[5]]
            make_traj(cur_pos, des_pos, 10)
            data = play_shared_traj(conn, data, 'traj.pkl', v, 10)
            if 'soft': v = args.neg_volt; send_arduino(comm_arduino, v)
            if 'rigid': send2gripper(conn_gripper, "o")
            timestep = time.time()
            first_loop = False

        if time.time() - timestep < 10:
            if wrench[2] > -10: x_dot[2] = -0.01
            else: x_dot[2] = 0
            x_dot[0] = z[1]
            x_dot[1] = z[0]

        if time.time() - timestep > 10: 
            shared = False
            drop = True
            print('dropping object in basket')

    # Once object picked up, continue to use Shared Autonomy Mode and drop object in basket
    if drop:
        basket_pos = [0.45, -0.65, 0.25, cur_pos[3], cur_pos[4], cur_pos[5]]
        make_traj(cur_pos, basket_pos, 10)
        data = play_shared_traj(conn, data, 'traj.pkl', v, 10)
        drop = False
        choose = True
        print('object dropped in basket')

    # Press to end trial
    if STOP_pressed:
        pickle.dump(data, open(file_name, 'wb'))
        print("DATA SAVED")
        # send_arduino(comm_arduino, args.neg_volt)
        break

    run_xdot(x_dot, conn)
    data = append_data(data, cur_pos, wrench, v, Joystick_inputs)
