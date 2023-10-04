from operator import truediv
import numpy as np
import cv2
import time
import pickle
import socket
from scipy.interpolate import interp1d
import pygame
import pygame.camera
import pyrealsense2 as rs
from tkinter import *



# Append data to the save dataset
def append_data(data, timestamp, cur_pos, wrench, voltage, Joystick_inputs):
    data["Time"].append(timestamp)
    data["Position"].append(cur_pos)
    data["Force"].append(wrench)
    data["Inputs"].append(Joystick_inputs)
    data["Voltage"].append(voltage)
    return data


def send_arduino(comm_arduino, user_input):
    string = '<' + str(user_input) + '>'
    comm_arduino.write(string.encode())


class Joystick(object):

    def __init__(self):
        pygame.init()
        self.gamepad = pygame.joystick.Joystick(0)
        self.gamepad.init()
        self.deadband = 0.1
        self.timeband = 0.5
        self.lastpress = time.time()

    def input(self):
        pygame.event.get()
        curr_time = time.time()
        z1 = self.gamepad.get_axis(0)
        z2 = self.gamepad.get_axis(1)
        z3 = self.gamepad.get_axis(4)
        if abs(z1) < self.deadband:
            z1 = 0.0
        if abs(z2) < self.deadband:
            z2 = 0.0
        if abs(z3) < self.deadband:
            z3 = 0.0
        A_pressed = self.gamepad.get_button(0) and (curr_time - self.lastpress > self.timeband)
        B_pressed = self.gamepad.get_button(1) and (curr_time - self.lastpress > self.timeband)
        X_pressed = self.gamepad.get_button(2) and (curr_time - self.lastpress > self.timeband)
        Y_pressed = self.gamepad.get_button(3) and (curr_time - self.lastpress > self.timeband)
        START_pressed = self.gamepad.get_button(7) and (curr_time - self.lastpress > self.timeband)
        STOP_pressed = self.gamepad.get_button(6) and (curr_time - self.lastpress > self.timeband)
        Right_trigger = self.gamepad.get_button(5)
        Left_Trigger = self.gamepad.get_button(4)
        if A_pressed or START_pressed or B_pressed:
            self.lastpress = curr_time
        return [z1, z2, z3], A_pressed, B_pressed, X_pressed, Y_pressed, START_pressed, STOP_pressed, Right_trigger, Left_Trigger


class Trajectory(object):

    def __init__(self, xi, T):
        """ create cublic interpolators between waypoints """
        self.xi = np.asarray(xi)
        self.T = T
        self.n_waypoints = xi.shape[0]
        timesteps = np.linspace(0, self.T, self.n_waypoints)
        self.f1 = interp1d(timesteps, self.xi[:,0], kind='cubic')
        self.f2 = interp1d(timesteps, self.xi[:,1], kind='cubic')
        self.f3 = interp1d(timesteps, self.xi[:,2], kind='cubic')
        self.f4 = interp1d(timesteps, self.xi[:,3], kind='cubic')
        self.f5 = interp1d(timesteps, self.xi[:,4], kind='cubic')
        self.f6 = interp1d(timesteps, self.xi[:,5], kind='cubic')

    def get(self, t):
        """ get interpolated position """
        if t < 0:
            q = [self.f1(0), self.f2(0), self.f3(0), self.f4(0), self.f5(0), self.f6(0)]
        elif t < self.T:
            q = [self.f1(t), self.f2(t), self.f3(t), self.f4(t), self.f5(t), self.f6(t)]
        else:
            q = [self.f1(self.T), self.f2(self.T), self.f3(self.T), self.f4(self.T), self.f5(self.T), self.f6(self.T)]
        return np.asarray(q)



""" Get locations of all objects in the environment"""
def get_targets():
    pipeline = configure_camera()
    align_to = rs.stream.color
    align = rs.align(align_to)

    try:
        depth_image, gray_img, gray1 = process_frames(pipeline, align)
        xc, yc, z = identify_objects(gray_img, gray1, depth_image)
        return xc, yc, z

    finally:
        pipeline.stop()


def configure_camera():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    # device.hardware_reset()

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    return pipeline


def process_frames(pipeline, align):
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    depth_image = depth_image[60:480, 0:640]
    color_image = np.asanyarray(color_frame.get_data())
    # Convert image to black and white
    gray_img = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    gray_img = gray_img[60:480, 0:640]

    gray_upper = 110
    gray_lower = 0

    kernal = np.ones((2, 2), "uint8")

    gray1 = cv2.inRange(gray_img, gray_lower, gray_upper)
    gray1 = cv2.morphologyEx(gray1, cv2.MORPH_OPEN, kernal)

    return depth_image, gray_img, gray1


def identify_objects(gray_img, gray1, depth_image):
    xc = []
    yc = []
    z = []
    
    # Find all objects
    (contoursred1, hierarchy) = cv2.findContours(gray1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contourred in enumerate(contoursred1):
        area = cv2.contourArea(contourred)
        if (area > 150):
            x, y, w, h = cv2.boundingRect(contourred)

            # SWAP X AND Y HERE
            xc.append(y + h/2)
            yc.append(x + w/2)
            z.append(depth_image[round(y+h/2), round(x+w/2)])

            gray_img = cv2.rectangle(gray_img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(gray_img, "OBJECT", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255))
    cv2.imwrite('image.jpg', gray_img)
    return np.array(xc), np.array(yc), np.array(z)


# convert camera view to robot's base frame
def convert_camera(xc, yc, z, obj):
    m_to_pixel = 0.52/420
    xc = xc*m_to_pixel + 0.22

    m_to_pixel = 0.762/640
    yc = yc*m_to_pixel - 0.35

    m_to_pixel = 0.76/755
    z = z*m_to_pixel # go slightly above object

    objects = []
    for idx in range(len(yc)):
        if z[idx] == 0:
            z[idx] = 0.71

        if obj == 'soft':
            objects.append([xc[idx], yc[idx] + 0.07, 0.76 - z[idx] + 0.05])
        if obj == 'rigid':
            objects.append([xc[idx], yc[idx], 0.76 - z[idx] + 0.05])
    return objects


# returns alpha value based on probability distribution
def get_alpha(P):
    val = 1/len(P) + 0.2
    if len(P) < 4: val = 0.5

    if np.max(P) > val:
        return 0.5
    else:
        return 0.0


# predict probability distribution
def predict_goal(s0, st, aH, THETA, prior, beta = 15):
    P = [0.] * len(THETA)
    # st += 0.5*aH

    for idx in range(len(THETA)):
        P[idx] = prior[idx]
    dist_so_far = np.linalg.norm(st - s0)

    for idx, theta in enumerate(THETA):
        dist_to_goal = np.linalg.norm(theta - st)
        min_dist = np.linalg.norm(theta - s0)
        num = np.exp(beta * min_dist)
        den = np.exp(beta * (dist_so_far + dist_to_goal))
        P[idx] += num/den

    P = np.asarray(P)
    return P / np.sum(P)


# move towards object with highest probability of being picked
def get_assist(s, THETA, P):
    idx = np.argmax(P)
    aR = THETA[idx] - s
    for idx, theta in enumerate(THETA):
        distance = np.linalg.norm(s-theta)
        print(distance)
        if distance < 0.05:
            return np.array([0., 0., 0.])
    return aR


"""Connecting and Sending commands to robot"""
def connect2robot(PORT):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('172.16.0.3', PORT))
    s.listen()
    conn, addr = s.accept()
    return conn


def send2robot(conn, qdot, mode, traj_name=None, limit=1.0):
    if traj_name is not None:
        if traj_name[0] == 'q':
            # print("limit increased")
            limit = 1.0
    qdot = np.asarray(qdot)
    scale = np.linalg.norm(qdot)
    if scale > limit:
        qdot *= limit / scale
    send_msg = np.array2string(qdot, precision=5, separator=',', suppress_small=True)[1:-1]
    send_msg = "s," + send_msg + "," + mode + ","
    conn.send(send_msg.encode())


def connect2gripper(PORT):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('172.16.0.3', PORT))

    s.listen()
    conn, addr = s.accept()
    return conn


def send2gripper(conn, arg):
    send_msg = arg
    conn.send(send_msg.encode())


def listen2robot(conn):
    state_length = 7 + 7 + 7 + 6 + 42
    message = str(conn.recv(2048))[2:-2]
    state_str = list(message.split(","))
    for idx in range(len(state_str)):
        if state_str[idx] == "s":
            state_str = state_str[idx + 1:idx + 1 + state_length]
            break
    try:
        state_vector = [float(item) for item in state_str]
    except ValueError:
        return None
    if len(state_vector) is not state_length:
        return None
    state_vector = np.asarray(state_vector)
    state = {}
    state["q"] = state_vector[0:7]
    state["dq"] = state_vector[7:14]
    state["tau"] = state_vector[14:21]
    state["O_F"] = state_vector[21:27]
    # print(state_vector[21:27])
    state["J"] = state_vector[27:].reshape((7, 6)).T

    # get cartesian pose
    xyz_lin, R = joint2pose(state_vector[0:7])
    beta = -np.arcsin(R[2, 0])
    alpha = np.arctan2(R[2, 1] / np.cos(beta), R[2, 2] / np.cos(beta))
    gamma = np.arctan2(R[1, 0] / np.cos(beta), R[0, 0] / np.cos(beta))
    xyz_ang = [alpha, beta, gamma]
    xyz = np.asarray(xyz_lin).tolist() + np.asarray(xyz_ang).tolist()
    state["x"] = np.array(xyz)
    return state


def readState(conn):
    while True:
        state = listen2robot(conn)
        if state is not None:
            break
    return state


def find_pos(conn):
    state = readState(conn)
    return state['x'], state['O_F']


def xdot2qdot(xdot, state):
    J_pinv = np.linalg.pinv(state["J"])
    return J_pinv @ np.asarray(xdot)


def run_xdot(xdot, conn):
    state = readState(conn)
    qdot = xdot2qdot(xdot, state)
    send2robot(conn, qdot, 'v')


def joint2pose(q):
    def RotX(q):
        return np.array([[1, 0, 0, 0], [0, np.cos(q), -np.sin(q), 0], [0, np.sin(q), np.cos(q), 0], [0, 0, 0, 1]])

    def RotZ(q):
        return np.array([[np.cos(q), -np.sin(q), 0, 0], [np.sin(q), np.cos(q), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    def TransX(q, x, y, z):
        return np.array([[1, 0, 0, x], [0, np.cos(q), -np.sin(q), y], [0, np.sin(q), np.cos(q), z], [0, 0, 0, 1]])

    def TransZ(q, x, y, z):
        return np.array([[np.cos(q), -np.sin(q), 0, x], [np.sin(q), np.cos(q), 0, y], [0, 0, 1, z], [0, 0, 0, 1]])

    H1 = TransZ(q[0], 0, 0, 0.333)
    H2 = np.dot(RotX(-np.pi / 2), RotZ(q[1]))
    H3 = np.dot(TransX(np.pi / 2, 0, -0.316, 0), RotZ(q[2]))
    H4 = np.dot(TransX(np.pi / 2, 0.0825, 0, 0), RotZ(q[3]))
    H5 = np.dot(TransX(-np.pi / 2, -0.0825, 0.384, 0), RotZ(q[4]))
    H6 = np.dot(RotX(np.pi / 2), RotZ(q[5]))
    H7 = np.dot(TransX(np.pi / 2, 0.088, 0, 0), RotZ(q[6]))
    H_panda_hand = TransZ(-np.pi / 4, 0, 0, 0.2105)
    H = np.linalg.multi_dot([H1, H2, H3, H4, H5, H6, H7, H_panda_hand])
    return H[:, 3][:3], H[:, :3][:3]


def go2home(conn, h=None):
    if h is None:
        home = np.copy(HOME)
    else:
        home = np.copy(h)
    total_time = 35.0;
    start_time = time.time()
    state = readState(conn)
    current_state = np.asarray(state["q"].tolist())

    # Determine distance between current location and home
    dist = np.linalg.norm(current_state - home)
    curr_time = time.time()
    action_time = time.time()
    elapsed_time = curr_time - start_time

    # If distance is over threshold then find traj home
    while dist > 0.05 and elapsed_time < total_time:
        current_state = np.asarray(state["q"].tolist())

        action_interval = curr_time - action_time
        if action_interval > 0.005:
            # Get human action
            qdot = home - current_state
            # qdot = np.clip(qdot, -0.3, 0.3)
            send2robot(conn, qdot, "v")
            action_time = time.time()

        state = readState(conn)
        dist = np.linalg.norm(current_state - home)
        curr_time = time.time()
        elapsed_time = curr_time - start_time

    # Send completion status
    if dist <= 0.02:
        return True
    elif elapsed_time >= total_time:
        return False


def wrap_angles(theta):
    if theta < -np.pi:
        theta += 2*np.pi
    elif theta > np.pi:
        theta -= 2*np.pi
    else:
        theta = theta
    return theta


def make_traj(start_pos, des_pos, val):
    traj = []
    for idx in range(val):
        traj.append(start_pos + (1/val) * idx * (des_pos - start_pos))
    traj.append(des_pos)
    pickle.dump(traj, open('traj.pkl', 'wb'))


def play_shared_traj(conn, data, traj_name, voltage, total_time, alpha = 0.2):
    traj = np.array(pickle.load(open(traj_name, "rb")))
    traj = Trajectory(traj[:, :6], total_time)
    start_t = time.time()
    interface = Joystick()
    while True:
        z, A_pressed, B_pressed, X_pressed, Y_pressed, START_pressed, STOP_pressed, RT, LT = interface.input()
        Joystick_inputs = [z, A_pressed, B_pressed, X_pressed, Y_pressed, START_pressed, STOP_pressed, RT, LT]
        curr_t = time.time() - start_t
        state = readState(conn)
        x_des = traj.get(curr_t)
        x_curr = state['x']
        wrench = state['O_F']
        x_des[0] = (x_curr[0]+z[1])*alpha + x_des[0]*(1-alpha)
        x_des[1] = (x_curr[1]+z[0])*alpha + x_des[1]*(1-alpha)
        x_des[2] = (x_curr[2] + z[2]) * alpha + x_des[2] * (1 - alpha)
        x_des[3] = wrap_angles(x_des[3])
        x_des[4] = wrap_angles(x_des[4])
        x_des[5] = wrap_angles(x_des[5])
        xdot = 1 * (x_des - x_curr)
        xdot[3] = wrap_angles(xdot[3])
        xdot[4] = wrap_angles(xdot[4])
        xdot[5] = wrap_angles(xdot[5])

        qdot = xdot2qdot(xdot, state)
        send2robot(conn, qdot, 'v', traj_name)
        data = append_data(data, time.time(), x_curr, wrench, voltage, Joystick_inputs)
        if curr_t > total_time or wrench[2] < -5 or A_pressed:
            return data

