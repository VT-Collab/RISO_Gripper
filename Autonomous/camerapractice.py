from fileinput import filename
from tracemalloc import start
import numpy as np
import pickle
import matplotlib.pyplot as plt
import socket
import argparse
import cv2
import time
import os, sys
from multiprocessing import Process
from utils import connect2robot, go2home, readState, play_traj,\
	get_target, send_force, teleop, connect2pressure,\
	connect2gripper, send2gripper
HOME = [0.0, -0.0, -0.0, -1.28331, -0.0, 1.27004, 0.756409]

# PORT_robot = 8080
# print('[*] Connecting to low-level controller...')
# conn = connect2robot(PORT_robot)
# print("Connection Established")

#go2home(conn, h=HOME)
#print("Returning Home")

while True:
    input("Observe object")
    x, y, z, obj = get_target()
    print("X:", y,"Y:", x,"Z:", z, obj)
    # states = readState(conn)
    # print(states['x'])
    