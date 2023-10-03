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
                   send_arduino, xdot2qdot, Joystick, play_shared_traj,
                   predict_goal, get_assist, convert_camera, get_alpha, get_targets)

# HOME = [-0.016779, -0.137233, -0.016136, -0.988438, -0.008862,  0.871688, 0.758049]
# HOME = [-0.017614, -0.375982,  0.001655, -1.531199,  0.003311,  1.16403 , 0.778423]
# HOME = [-0.031721, -0.371688,  0.046463, -1.519194,  0.021286,  1.154843, 0.810934]
# HOME = [-0.027781, -0.374255,  0.066114, -1.525339,  0.026778,  1.158676, 0.829868]
HOME = [-0.005758, -0.363266,  0.131343, -1.509827,  0.055365,  1.158452, 0.923992]
os.system('clear')
# DEFINE PARAMETERS
p = argparse.ArgumentParser()
p.add_argument('--user', type=str, required=True, help='Type in the user number to save data')
args = p.parse_args()

file_name = 'user_study/user_{}.pkl'.format(args.user)

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

# Initialize Variables
interface = Joystick()
data = {"Time": [], "Position": [], "Force": [], "Inputs": [], "Voltage": []}
voltage = 8.1
send_arduino(comm_arduino, str(voltage))
send2gripper(conn_gripper, 'o')
time.sleep(2)

input('Press Enter to Start Trial')
scaleh = 0.5
choose = True
pick = False
des_force = -15


while True:

    # SHARED AUTON MODE: pick up object
    while True:
        z, A_pressed, B_pressed, X_pressed, Y_pressed, START_pressed, STOP_pressed, RT, LT = interface.input()
        Joystick_inputs = [z, A_pressed, B_pressed, X_pressed, Y_pressed, START_pressed, STOP_pressed, RT, LT]
        aH = np.array([z[1], z[0], -z[2]])
        cur_pos, wrench = find_pos(conn)
        x_dot = [0.] * 6
        data = append_data(data, time.time(), cur_pos, wrench, voltage, Joystick_inputs)
        
        if choose:
            if A_pressed or B_pressed:
                choose = False
                pick = True

                # first take picture of all objects
                xc, yc, z = get_targets()
                print(xc, yc, z)
                

                # give location of objects in robot's frame
                if A_pressed:
                    obj = 'soft'
                    voltage = 8.4
                    send_arduino(comm_arduino, str(voltage))
                    time.sleep(7)

                if B_pressed:
                    obj = 'rigid'
                    send2gripper(conn_gripper, "o")

                objects = convert_camera(xc, yc, z, obj)
                print(objects)
                # input()
                prior = [1/(len(objects))] * len(objects)
                timestep = time.time()

        if START_pressed:
            pickle.dump(data, open(file_name, 'wb'))
            print("DATA SAVED")
            send_arduino(comm_arduino, 8.1)
            exit()

        if pick:
            if time.time() - timestep > 0.5:
                P = predict_goal(data['Position'][0][0:3], cur_pos[0:3], aH[0:3], objects, prior)
                prior = P
                alpha = get_alpha(P)
                aR = get_assist(cur_pos[0:3], objects, P)
                print(P)
                x_dot[0] = (1-alpha)*scaleh*aH[0] + alpha*aR[0]
                x_dot[1] = (1-alpha)*scaleh*aH[1] + alpha*aR[1]
                x_dot[2] = (1-alpha)*scaleh*aH[2] + alpha*aR[2]
                run_xdot(x_dot, conn)

        
        if STOP_pressed:
            choose = True
            pick = False
            break

    # FULLY AUTON MODE: drop object in basket
    if obj == 'rigid':
        send2gripper(conn_gripper, "c")
        time.sleep(2)

    if obj == 'soft':
        flag = 1
        while True:
            cur_pos, wrench = find_pos(conn)
            x_dot = [0.] * 6
            x_dot[2] = -0.002*(wrench[2] - des_force)
            print(x_dot[2])

            if wrench[2] < -10 and flag:
                voltage = 3.0
                send_arduino(comm_arduino, str(voltage))
                flag = 0

            elif abs(x_dot[2]) < 0.0005:
                # print("OUT OF LOOP!!!!!!!!!!!!!!")
                timestep = time.time()
                while time.time() - timestep < 7:
                    x_dot = [0.] * 6
                    x_dot[2] = -0.002
                    run_xdot(x_dot, conn)
                break
            run_xdot(x_dot, conn)
            

    timestep = time.time()
    while time.time() - timestep < 4:
        xdot = [0., 0., 0.1, 0., 0., 0.]
        run_xdot(xdot, conn)

    cur_pos, wrench = find_pos(conn)
    basket_pos = [0.45, -0.68, 0.28, cur_pos[3], cur_pos[4], cur_pos[5]]
    make_traj(cur_pos, basket_pos, 5)
    data = play_shared_traj(conn, data, 'traj.pkl', voltage, 5)
    
    if obj == 'rigid':
        send2gripper(conn_gripper, 'o')
        time.sleep(2)

    if obj == 'soft':
        voltage = 8.4
        send2gripper(conn_gripper, 'o')
        time.sleep(2)
        send_arduino(comm_arduino, voltage)
        time.sleep(10)

    print('object dropped in basket')

    if obj == 'soft':
        voltage = 2.0
        send_arduino(comm_arduino, voltage)

    go2home(conn, h=HOME)
    print('ready to pick up next object')
