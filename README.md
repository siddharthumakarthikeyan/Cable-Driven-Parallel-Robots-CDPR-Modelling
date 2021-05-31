# Modelling and Implementation of Cable Driven Parallel Manipulator System with Tension Control
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT) [![Build Status](https://travis-ci.org/saimouli/frontier_exploration_turtlebot.svg?branch=master)](https://travis-ci.org/saimouli/frontier_exploration_turtlebot) [![Coverage Status](https://coveralls.io/repos/github/saimouli/frontier_exploration_turtlebot/badge.svg?branch=master)](https://coveralls.io/github/saimouli/frontier_exploration_turtlebot?branch=master)

Cable Driven Parallel Robots (CDPR) is also known as Cable-Suspended Robots are the emerging and flexible end effector manipulation system. Cable-driven parallel robots (CDPRs) are categorized as a type of parallel manipulators. In CDPRs, flexible cables are used to take the place of rigid links. The particular property of cables provides CDPRs several advantages, including larger workspaces, higher payload-to-weight ratio and lower manufacturing costs rather than rigid-link robots. In this paper, the history of the development of CDPRs is introduced and several successful latest application cases of CDPRs are presented. The theory development of CDPRs is introduced focusing on design, performance analysis and control theory. Research on CDPRs gains wide attention and is highly motivated by the modern engineering demand for large load capacity and workspace. A number of exciting advances in CDPRs are summarized in this paper since it is proposed in the 1980s, which points to a fruitful future both in theory and application. In order to meet the increasing requirements of robot in different areas, future steps foresee more in-depth research and extension applications of CDPRs including intelligent control, composite materials, integrated and reconfigurable design.


The robot geometry is defined in a YAML file (see sdf folder), then generates a SDF file through the call to `gen_cdpr.py <file>.yaml`.

The same robot can be simulated by calling `roslaunch cdpr cdpr.launch model:=<robot model>` where `robot model` corresponds to the YAML and SDF files.

A very basic PID controller can be tested using `rosrun cdpr pid_control`. The controller does not take into account the positive-only tensions and is just here to show the use of the CDPR class that interfaces with Gazebo.

It is also possible to run both the simulation and the controller using the lunch files in the cdpr_controllers package for instance `roslaunch cdpr_controllers cdpr_pid_qp.launch`.


### Model generation

An example is given through cdpr/sdf/caroca.yaml. The sim_cables field leads to two behaviors: 
* if True then Gazebo wil simulate the cables as rigid bodies and subscribe for cable tensions
* if False then Gazebo will simulate a free-floating platform and subscribe for cdpr::Tensions which are the tensions + unit vector of all cables.


## Installation

_tested with ROS 1 noetic under Ubuntu 20.04_


### Building from source in a catkin workspace

```
mkdir catkin_ws
cd catkin_ws
git clone https://github.com/remipannequin/cdpr.git src
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```


### Some works
The improvement with using CTC control algorithm and trajectory generator

The control algorithm is CTC which integrates the quadratic programming optimization method in order to get the feasible tension in cables.

The trajectory parameter is defined by `trajectory.yaml` in sdf folder.

The SDF file is loaded by the launch file directly when using `roslaunch` command to launch one controller.

The 5 orders polynomial is implemented to generate one trajectory with updating time 0.01. It is set the same initial pose with initialization of cdpr, and the desired position is [2,2,1].


## 1. Rx, Ry & Rz Speed Simulation

### (I) Rx Speed
![rx_gif](https://user-images.githubusercontent.com/82173562/120223007-85a79980-c25e-11eb-87f1-3ad15f5e47bc.gif)

### (II) Ry Speed
![yr_gif](https://user-images.githubusercontent.com/82173562/120223037-90fac500-c25e-11eb-82e4-6188dbfeddd1.gif)

### (III) Rz Speed
![rz_gif](https://user-images.githubusercontent.com/82173562/120223070-9d7f1d80-c25e-11eb-8b5b-37e3bedde517.gif)



## 2. Movement of the Suspended Platform

This robot can move in 3D space and can achieve any 6 degrees of freedom. 
![ezgif com-gif-maker](https://user-images.githubusercontent.com/82173562/120223101-aa9c0c80-c25e-11eb-8d24-24df836e91f2.gif)



## 3. Gripping a Load 

![grip](https://user-images.githubusercontent.com/82173562/120223131-bbe51900-c25e-11eb-8b14-de41690363e0.gif)


