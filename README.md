# 4 DOF Robotic Arm

This repository contains the source code, documentation, and design files for a 4 degree-of-freedom (DOF) robotic arm that performs a pick and place task in a A4 working envelope. The arm is controlled using Python scripts, integrated with the MoveIt framework and simulated with Gazebo Simulation.

## Table of Contents

- [4 DOF Robotic Arm](#4-dof-robotic-arm)
  - [Overview](#overview)
  - [Features](#features)
  - [Installation](#installation)
  - [Usage](#usage)

## Overview

The 4 DOF Robotic Arm project aims to design and implement a flexible and programmable robotic arm with four degrees of freedom. It utilizes Python scripts for control and integrates with the MoveIt framework, which provides motion planning, kinematics, and collision checking capabilities. This repository serves as a central hub for all project-related resources.

## Features

- **4 degrees of freedom:** The robotic arm consists of four independently controllable joints, allowing it to move in multiple directions and perform complex tasks. three of the joints are controlled using servo motors and the last joint is actuated using a DC motor while using PID controller for the position control task.
- **Python control scripts:** The arm is controlled using Python scripts, providing flexibility and ease of integration with other systems.
- **MoveIt integration:** MoveIt is a widely used robotics motion planning framework that provides advanced motion planning algorithms, kinematics, and collision checking. The robotic arm is integrated with MoveIt for seamless motion planning and control.
- **Simulation support:** The arm can be simulated in popular robotics simulation environments such as Gazebo and RViz, enabling testing and development without the need for physical hardware.
- **Real world Interface:** The provided codes have undergone extensive testing in real-world scenarios using a 4 DOF arm controlled by an ATMEGA16 microcontroller.
## Installation

To set up the 4 DOF Robotic Arm project, follow these steps:
1. Install the necessary dependencies:

   - [MoveIt](https://moveit.ros.org/install/): Install MoveIt motion planning framework following the instructions specific to your operating system.
      ```shell
     sudo apt-get update    
     sudo apt install ros-noetic-moveit
     ```


   - ROS controllers: Install the ROS controllers package by running the following command:

     ```shell
     sudo apt-get install ros-noetic-ros-controllers
     ```

   - Make sure to install all the necessary dependencies to ensure proper functioning of the robotic arm. You can use [rosdep](http://wiki.ros.org/rosdep) for this task.

2. Clone the repository:

   ```shell
   git clone https://github.com/OmarRamzy45/4DOF_Robotic_Arm.git
   ```

3. build the packages in your workspace directory:
   ```shell
   cd <catkin_ws>
   source devel/setup.bash
   catkin build
   ```
## Usage

1. Start Rviz fo visualization:
   ```shell
   cd <catkin_ws>
   source devel/setup.bash
   roslaunch robot_moveit demo.launch
   ```
2. Start robot planning: 
   ```shell
   cd <catkin_ws>/src/robot_moveit/src
   python3 main.py
   ```
3. Start rosserial nodes to move your robot:
   ```shell
   cd <catkin_ws>
   source devel/setup.bash
   roslaunch robot_moveit rosserial.launch
    ```   
