# Unified Rigid and Soft Grippers for Manipulating Objects during Human-Robot Interaction

This repo porivdes our implementation of Human Control, Autonomous Control and Shared Control for leveraging RISO grippers with 7-DoF Franka Emika Panda robot arm.

You need to have the following libraries with [Python3](https://www.python.org/):

- [Numpy](https://numpy.org/)
- [CV2](https://pypi.org/project/opencv-python/)
- [SciPy](https://scipy.org/)
- [pygame](https://www.pygame.org/news)
- [pyrealsense2](https://pypi.org/project/pyrealsense2/)

## Install 
To install this repo use the command 

```bash
git clone https://github.com/VT-Collab/RISO_ScienceRobotics.git
```
## Run Human Control
```bash
cd /home/<user>/RISO_ScienceRobotics/Human
python3 fully_remote.py --gripper <gripper_type> --user test
```
## Run Autonomous Control
```bash
cd /home/<user>/RISO_ScienceRobotics/Autonomous
python3 fully_auton.py --gripper <gripper_type> --des_force -25
```

## Run Shared Control
```bash
cd /home/<user>/RISO_ScienceRobotics/Shared
python3 bayesian_code.py --user test
```