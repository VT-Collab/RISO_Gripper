# Unified Rigid and Soft Grippers for Manipulating Objects during Human-Robot Interaction

This repo porivdes our implementation of Human Control, Autonomous Control and Shared Control for leveraging RISO grippers with 7-DoF Franka Emika Panda robot arm.

You need to have the following libraries with [Python3](https://www.python.org/):

- [Numpy](https://numpy.org/)
- [CV2](https://pypi.org/project/opencv-python/)
- [SciPy](https://scipy.org/)
- [pygame](https://www.pygame.org/news)
- [pyrealsense2](https://pypi.org/project/pyrealsense2/)

For controlling the pressure regulators, you need the following modules for [Arduino](https://www.arduino.cc/)
- [Adafruit 12-bit I2C DAC](https://www.adafruit.com/product/935)
- [Adafruit MCP4725 Library](https://github.com/adafruit/Adafruit_MCP4725)
- [DFRobot Gravity:2-channel I2C DAC](https://www.dfrobot.com/product-2613.html)
- [DFRobot_GP8403 Library](https://github.com/DFRobot/DFRobot_GP8403)

## Install 
To install this repo use the command 

```bash
git clone https://github.com/VT-Collab/RISO_ScienceRobotics.git
```
## Human Control
In this control approach, the user is in complete control of the robot and the gripper. The user operates the gripper and the robot using a handheld controller. 

For the RISO gripper, the user controls the rigid gripper as well as the pressure in the soft adhesives for soft grasping. For the Granular gripper [[1]](#references) and SoftGripper [[2]](#references), the user only controls the pressure in the gripper. For ease of controlling the pressure and force that the gripper applies, we provide a GUI that shows the pressure and force values to the user when performing the task.

The approach can be implemented by running the following code:

```bash
cd /home/<path_to_dir>/RISO_ScienceRobotics/Human
python3 fully_remote.py --gripper <gripper_type> --user test
```
Replace the <gripper_type> with 'riso', 'granular' or 'modular' based on the gripper being used and <path_to_dir> with your folder path to this directory

## Autonomous Control
Using this control strategy, the robot autonomously detects the objects and plans its trajectory to perform the grasp without any human feedback.

The robot leverages an [Intel RealSense D435](https://www.intelrealsense.com/depth-camera-d435/) depth camera to detect the objects present in its workspace. It determines the object to grasp based on the area of the object, i.e., the robot picked up the object with the largest area first. The grasp type for RISO was determined based on the height of the object to grasp. If the height if the object was greater that 7.5 cm, the robot used the rigid mechanism for the grasp, and soft adhesives otherwise.

For all the soft grasps, i.e. for RISO (soft adhesive), Granular gripper [[1]](#references) and SoftGripper [[2]](#references), the robot used preset pressure values to perform the grasps. We performed multiple tests with different pressure values and selected the best performing pressure values for performing the autonomous tests.

The approach can be implemented by running the following code:

```bash
cd /home/<path_to_dir>/RISO_ScienceRobotics/Autonomous
python3 fully_auton.py --gripper <gripper_type> --des_force -25 --user test
```

Replace the <gripper_type> with 'riso', 'granular' or 'modular' based on the gripper being used and <path_to_dir> with your folder path to this directory


## Shared Control
Here, we implement the shared control algorithm outlined in Section 2.4 of the manuscript.

Leveraging the RGB-D camera mounted at its end-effector, the robot gets the positions of all the objects in its workspace. The user inputs the grasp type to the robot using the controller before starting the experiment. As the user starts moving the robot, the robot uses Equation (2):

$$
P(o, g \mid s^{0:t}, a_\mathcal{H}^{0:t}) = b^{t+1}(o, g) \propto P(a_\mathcal{H}^t \mid s^t, o, g) \cdot b^t(o, g)
$$

to update its belief over the object the user is trying to reach. Once the robot is confident about the object the human is reaching for, the robot starts taking actions to assist the user based on Equation (3):

$$
a_\mathcal{R} = \sum_{o \in \mathcal{O}} \sum_{g \in \mathcal{G}} (o - s_g) \cdot b(o, g)
$$

We get the final actions to be taken by blending the robot actions with the human actions as $a = \alpha \cdot a_\mathcal{H} + (1 - \alpha) \cdot a_\mathcal{R}$

Once the robot reaches the objects and the user completes the final alignment of the gripper with the object, the robot takes over and performs the grasp using the grasp type selected by the user. For the soft grasps using RISO the robot uses preset pressure and force values similar to [Autonomous Control](#autonomous-control). 

The approach can be implemented by running the following code:


```bash
cd /home/<path_to_dir>/RISO_ScienceRobotics/Shared
python3 bayesian_code.py --user test
```
Replace <path_to_dir> with your folder path to this directory

## References

[[1]](https://www.soft-gripping.shop/en/robot-magician-softgripper-3-fingers.html) Dobot Magician SoftGripper - 3 Fingers

[[2]](https://www.pnas.org/doi/abs/10.1073/pnas.1003250107) Eric Brown et  el. “Universal robotic gripper based on the jamming of granular material”. In:Proceedings of the National Academy of Sciences 107.44 (2010), pp. 18809–18814.
