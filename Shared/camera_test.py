import numpy as np
import cv2
# from cv2 import *
from utils import (append_data, connect2gripper, connect2robot, find_pos,
				   go2home, joint2pose, make_traj, run_xdot, send2gripper,
				   send_arduino, xdot2qdot, Joystick, play_shared_traj,
				   predict_goal, get_assist, convert_camera, get_alpha, get_targets)

import pyrealsense2 as rs

xc,yc,z = get_targets()