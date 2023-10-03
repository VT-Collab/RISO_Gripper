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
                   go2home, joint2pose, make_traj, run_xdot, send2gripper,
                   send_arduino, xdot2qdot, Joystick, play_traj, send2robot,xdot2qdot)

HOME = [0.163245, -0.283813,  0.028734, -2.538687,  0.038531,  2.277892, 0.937299]

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

# CONNECT TO PRESSURE PORT
comm_arduino = serial.Serial('/dev/ttyACM0', baudrate=115200)
print('Connected to Arduino')

# GO TO HOME POSITION
print("Returning Home")
go2home(conn, h=HOME)

voltage = 8.16
max_volt = 8.4 # To release
min_volt = 0.0 # To grasp
des_force = -15
grmod = 5
interface = Joystick()
action_scale = 0.3
print('[*] Ready for a teleoperation...')
timestep = time.time()
data = 0
while True:
    z, A_pressed, B_pressed, X_pressed, Y_pressed, START_pressed, STOP_pressed, RT, LT = interface.input()
    cur_pos, wrench = find_pos(conn)
    x_dot = [0]*6

    # GO ABOVE PLATE
    if START_pressed:
        go2home(conn, h=HOME)

    # OPEN/CLOSE GRIPPER
    if X_pressed and (time.time() - timestep) > 1:
        send2gripper(conn_gripper, "c")
        timestep = time.time()

    if Y_pressed and (time.time() - timestep) > 1:
        send2gripper(conn_gripper, "o")
        timestep = time.time()

    # POUR TOMATO SAUCE
    if A_pressed:
        start_pos = cur_pos
        des_pos = [cur_pos[0], cur_pos[1], cur_pos[2], cur_pos[3], cur_pos[4] + 1.5, cur_pos[5]]
        make_traj(start_pos, des_pos, 5)
        data = play_traj(conn, data, 'traj.pkl', 0, 5)
        des_pos, wrench = find_pos(conn)
        make_traj(des_pos, start_pos, 5)
        data = play_traj(conn, data, 'traj.pkl', 0, 5)

    # POUR CHEESE
    if B_pressed:
        start_pos = cur_pos
        des_pos = [cur_pos[0], cur_pos[1], cur_pos[2], cur_pos[3], cur_pos[4] + 0.3, cur_pos[5]]
        make_traj(start_pos, des_pos, 3)
        data = play_traj(conn, data, 'traj.pkl', 0, 3)

    # PICK OBJECT  
    if LT:
        send_arduino(comm_arduino, 8.4)

    if RT:
        flag = 1
        while True:
            cur_pos, wrench = find_pos(conn)
            x_dot = [0.] * 6
            x_dot[2] = -0.002*(wrench[2] - des_force)
            print(x_dot[2])

            if wrench[2] < (des_force/2) and flag:
                send_arduino(comm_arduino, 3.0)
                flag = 0

            elif abs(x_dot[2]) < 0.0005:
                # print("OUT OF LOOP!!!!!!!!!!!!!!")
                timestep = time.time()
                while time.time() - timestep < 7:
                    x_dot = [0.] * 6
                    x_dot[2] = -0.0001
                    run_xdot(x_dot, conn)
                print('object picked')
                break
            run_xdot(x_dot, conn)

    x_dot[0] = action_scale * z[1]
    x_dot[1] = action_scale * z[0]
    x_dot[2] = action_scale * -z[2]
    run_xdot(x_dot, conn)
