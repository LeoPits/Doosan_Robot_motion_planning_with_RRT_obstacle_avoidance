# Doosan Robot Movement with RRT* Algorithm and Obstacle Avoidance

This repository contains code and documentation for controlling a **Doosan Robot** using the **RRT* (Rapidly-exploring Random Tree Star)** algorithm for path planning and obstacle avoidance. The objective is to navigate the robot through an environment while avoiding obstacles, ensuring efficient and safe movement.

## Project Overview

The Doosan robot is equipped with sensors to detect obstacles in its environment. This project leverages the RRT* algorithm to generate an optimal path from a start point to a goal point while dynamically avoiding obstacles in the robot’s path.

### Key Features
- **RRT* Algorithm**: A variation of the RRT algorithm that optimizes the path by continuously improving it, ensuring a more efficient solution compared to standard RRT.
- **Obstacle Avoidance**: The robot detects obstacles in real-time and adjusts its path accordingly, ensuring smooth navigation through the environment.
- **Real-time Simulation**: The system updates the robot’s path dynamically based on the current state of the environment.

## Requirements

To run the project, you will need the following:

- **Python 3.x**
- **ROS (Robot Operating System)** - For controlling the Doosan robot and simulating the movement.
- **NumPy** - For mathematical calculations.
- **Matplotlib** - For visualizing the path planning and obstacles.
- **OpenCV** - For processing sensor data and obstacle detection.
- **Doosan SDK** (if applicable) - To interface with the Doosan robot hardware.

### Installing Dependencies
https://github.com/doosan-robotics/doosan-robot

