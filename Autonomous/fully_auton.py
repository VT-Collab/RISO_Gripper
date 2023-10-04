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

# Define Start Position
HOME = [0.0, -0.0, -0.0, -1.28331, -0.0, 1.27004, 0.756409]

# DEFINE PARAMETERS
parser = argparse.ArgumentParser()
parser.add_argument('--des_force', type=int, default=-25)

# Gripper Types: Modular, Granular, Soft
parser.add_argument('--gripper', type=str, default='granular')
args = parser.parse_args()
t = time.localtime()
current_time = time.strftime("%H:%M:%S", t)
file_name = 'user study: autonomous ' + args.gripper + ' ' + current_time + '.pkl'

# CONNECT TO ROBOT
PORT_robot = 8080
print('[*] Connecting to low-level controller...')
conn = connect2robot(PORT_robot)
print("Connection Established")

# GO TO HOME POSITION
print("Returning Home")
go2home(conn, h=HOME)

step = 0
data = {"Time": [], "Position": [], "Force": [], "Inputs": [], "Voltage": []}
if args.gripper == 'granular':
    # SET PRESSURE VALUES
    pos_volt = '6.0'
    neg_volt = '0.0'
    # CONNECT TO PRESSURE
    comm_arduino1 = serial.Serial('/dev/ttyACM2', baudrate=9600)
    comm_arduino2 = serial.Serial('/dev/ttyACM3', baudrate=19200)

    send_arduino(comm_arduino1, '1')
    send_arduino(comm_arduino2, pos_volt)
elif args.gripper == 'modular':
    # SET PRESSURE VALUES
    pos_volt = '0.0'
    neg_volt = '14.0'
    # CONNECT TO PRESSURE
    comm_arduino1 = serial.Serial('/dev/ttyACM2', baudrate=9600)
    comm_arduino2 = serial.Serial('/dev/ttyACM3', baudrate=19200)

    send_arduino(comm_arduino1, '0')
    send_arduino(comm_arduino2, pos_volt)
elif args.gripper == 'soft':
    # SET PRESSURE VALUES
    pos_volt = 8.4
    neg_volt = 4
    # CONNECT TO PRESSURE
    comm_arduino2 = serial.Serial('/dev/ttyACM1', baudrate=115200)
    # CONNECT TO GRIPPER
    PORT_gripper = 8081
    print("[*] Connecting to gripper")
    conn_gripper = connect2gripper(PORT_gripper)
    print("Connected to gripper")

    send_arduino(comm_arduino2, pos_volt)

time.sleep(15)
print('Connected to Arduino')
input("Enter to start test")

while True:
    P = input("Are there objects to transport? Y or N: ")
    
    # Pick up object
    if P == 'Y':
        # Find start position
        start_pos, start_q, states = find_pos(conn)
        # Detect the desired object
        y, x, z, obj = get_target()

        # preset gripper type when using other grippers
        if args.gripper == 'granular':
            obj = 'soft' 
        elif args.gripper == 'modular':
            obj = 'rigid'

        print(x, y, z, obj)
        des_pos = np.array([x, y, z+0.2, start_pos[3], start_pos[4], start_pos[5]])

        # if error in camera recording, preset height
        if z == 0.755 or z == 0.735:
            des_pos = np.array([x, y, 0.2, start_pos[3], start_pos[4], start_pos[5]])

        # go to desired object
        make_traj(start_pos, des_pos, 10)
        data = play_traj(conn, data, 'traj.pkl', pos_volt, 10)
        print('traveled to desired object')
        start_pos = des_pos

        # pick up desired object
        if obj == 'rigid': 
            if args.gripper == 'soft':
                send2gripper(conn_gripper, "o")
                
            des_pos = np.array([x, y, z-0.8*z, start_pos[3], start_pos[4], start_pos[5]])
            if z == 0.755 or z == 0.735:
                des_pos = np.array([x, y, 0., start_pos[3], start_pos[4], start_pos[5]])
            make_traj(start_pos, des_pos, 10)
            data = play_traj(conn, data, 'traj.pkl', pos_volt, 10)

            if args.gripper == 'modular':
                send_arduino(comm_arduino1, '1')
                send_arduino(comm_arduino2, neg_volt)
                time.sleep(2)
            elif args.gripper == 'soft':
                send2gripper(conn_gripper, "c")
                time.sleep(2)

        if obj == 'soft':
            des_pos = np.array([x, y, 0.0, start_pos[3], start_pos[4], start_pos[5]])
            if z == 0.755 or z == 0.735:
                des_pos = np.array([x, y, 0.00 , start_pos[3], start_pos[4], start_pos[5]])
            make_traj(start_pos, des_pos, 10)
            data = play_traj(conn, data, 'traj.pkl', pos_volt, 10)

            if args.gripper == 'granular':
                send_arduino(comm_arduino1, '0')
                send_arduino(comm_arduino2, neg_volt)
            elif args.gripper == 'soft':
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

        # lift straight up for 10 seconds to avoid other objects
        timestep = time.time()
        while (time.time() - timestep) < 10:
            xdot = [0., 0., 0.02, 0., 0., 0.]
            run_xdot(xdot, conn)
        
        # move to basket
        start_pos, start_q, states = find_pos(conn)
        basket_pos = [0.45, -0.65, 0.25, start_pos[3], start_pos[4], start_pos[5]]
        make_traj(start_pos, basket_pos, 10)
        data = play_traj(conn, data, 'traj.pkl', pos_volt, 10)

        # place object in basket
        if obj == 'rigid':
            if args.gripper == 'soft':
                send2gripper(conn_gripper, "o")
                time.sleep(2)
            elif args.gripper == 'modular':
                send_arduino(comm_arduino1, '0')
                send_arduino(comm_arduino2, pos_volt)
            
        if obj == 'soft':
            if args.gripper == 'soft':
                send_arduino(comm_arduino2, pos_volt)
                time.sleep(15)
            elif args.gripper == 'granular':
                send_arduino(comm_arduino1, '1')
                send_arduino(comm_arduino2, pos_volt)
                time.sleep(15)

        print('object dropped in basket')

        # go to home position
        go2home(conn, h=HOME)
        
    if P == 'N':
        print("Congrats!! You're done")
