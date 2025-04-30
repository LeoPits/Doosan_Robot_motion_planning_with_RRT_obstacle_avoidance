# Doosan Robot Movement with RRT* Algorithm and Obstacle Avoidance

This repository contains code and documentation for controlling a **Doosan Robot** using the **RRT* (Rapidly-exploring Random Tree Star)** algorithm for path planning and obstacle avoidance. The objective is to navigate the robot through an environment while avoiding obstacles, ensuring efficient and safe movement.

## Project Overview

The Doosan robot is equipped with sensors to detect obstacles in its environment. This project leverages the RRT* algorithm to generate an optimal path from a start point to a goal point while dynamically avoiding obstacles in the robot’s path.

### Key Features
- **RRT* Algorithm**: A variation of the RRT algorithm that optimizes the path by continuously improving it, ensuring a more efficient solution compared to standard RRT.
- *Obstacle Avoidance*: The robot know enviroment and the  obstacles  and adjusts its path accordingly 
- **STOMP (Stochastic Trajectory Optimization for Motion Planning)* optimizer is implemented to guarantee smooth navigation through the environment.


## Requirements

To run the project, you will need the following:

- **Python 3.x**
- **ROS (Robot Operating System)** - For controlling the Doosan robot and simulating the movement.
- **Doosan SDK** (if applicable) - To interface with the Doosan robot hardware. https://github.com/doosan-robotics/doosan-robot
NOTE: require Docker install for emulator sh. In src/doosan-robot/common/bin/DRCF folder
 ```
./run_dcfc.sh
 ```
to install the doosan emualtor on the device, which is necessary for controlling the robot



https://github.com/user-attachments/assets/59e11b56-9a22-4244-8e8d-b79b389e9623






## System requirements
- ubuntu 20.04 lts
- ros noetic


Launch the simulation with RViz: 
```
roslaunch dsr_launcher single_robot_rviz.launch
 ```

 Initialize the controller node to move the robot
 ```
 rosrun dsr_example_py __main.py
```
⚙️ Note: The start position, target position, obstacles and other parameters  are defined directly in the "__main.py" script.
