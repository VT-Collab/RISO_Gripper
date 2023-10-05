import argparse
import os
import pickle
import time
import numpy as np
import serial
from utils import *

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

# Initialize save data format
data = {"Time": [], "Position": [], "Force": [], "Inputs": [], "Voltage": []}

# Initialize Variables
interface = Joystick()
voltage = 8.1
scaleh = 0.5
des_force = -15
send_arduino(comm_arduino, str(voltage))
send2gripper(conn_gripper, 'o')
time.sleep(2)
# Boolean variables for mode switching
choose = True
pick = False

input('Press Enter to Start Trial')

while True:

    # SHARED AUTONOMY MODE: pick up object
    
    while True:
        z, A_pressed, B_pressed, X_pressed, Y_pressed, START_pressed, STOP_pressed, RT, LT = interface.input()
        Joystick_inputs = [z, A_pressed, B_pressed, X_pressed, Y_pressed, START_pressed, STOP_pressed, RT, LT]
        aH = np.array([z[1], z[0], -z[2]])
        cur_pos, wrench = find_pos(conn)
        x_dot = [0.] * 6
        data = append_data(data, time.time(), cur_pos, wrench, voltage, Joystick_inputs)

        # before picking an object: choose gripper and get object positions
        if choose:
            if A_pressed or B_pressed:
                choose = False
                pick = True

                # get object positions in camera frame
                xc, yc, z = get_targets()
                print(xc, yc, z)

                # user input determines grasp type
                if A_pressed:
                    obj = 'soft'
                    voltage = 8.4
                    send_arduino(comm_arduino, str(voltage))
                    # wait for the pressure to build up
                    time.sleep(7)

                if B_pressed:
                    obj = 'rigid'
                    send2gripper(conn_gripper, "o")

                # get position of objects in robot's frame
                objects = convert_camera(xc, yc, z, obj)
                print(objects)

                # define robot's prior belief over objects
                prior = [1/(len(objects))] * len(objects)
                timestep = time.time()
        
        # shared motion towards object based on probability distribution
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
            
        # end trial
        if START_pressed:
            pickle.dump(data, open(file_name, 'wb'))
            print("DATA SAVED")
            send_arduino(comm_arduino, 8.1)
            exit()

    
    # after aligning with the target, robot takes over to pick objects using preset picking procedure
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
                timestep = time.time()
                while time.time() - timestep < 7:
                    x_dot = [0.] * 6
                    x_dot[2] = -0.002
                    run_xdot(x_dot, conn)
                break
            run_xdot(x_dot, conn)

    # move objects to the bin
    timestep = time.time()
    while time.time() - timestep < 4:
        xdot = [0., 0., 0.1, 0., 0., 0.]
        run_xdot(xdot, conn)

    cur_pos, wrench = find_pos(conn)
    basket_pos = [0.45, -0.68, 0.28, cur_pos[3], cur_pos[4], cur_pos[5]]
    make_traj(cur_pos, basket_pos, 5)
    data = play_shared_traj(conn, data, 'traj.pkl', voltage, 5)

    # release object 
    if obj == 'rigid':
        send2gripper(conn_gripper, 'o')
        time.sleep(2)

    if obj == 'soft':
        voltage = 8.4
        send_arduino(comm_arduino, voltage)
        time.sleep(10)

    print('object dropped in basket')

    # move to home location and reset soft gripper
    if obj == 'soft':
        voltage = 2.0
        send_arduino(comm_arduino, voltage)
    go2home(conn, h=HOME)
    print('ready to pick up next object')
