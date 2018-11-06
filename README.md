# ENPM808X - ROS Beginner Tutorial
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

# Overview
This repository contains basic ROS C++ subscriber and publisher taken from the ROS [website](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29). 
- Takler (src/talker.cpp): publisher
- Listener (src/listener.cpp): subscriber


A launch file named ```week10HW.launch``` can be used to launch both talker and listener nodes 

A service message named ```changeText.srv``` is used to change the output string upon request by the client


# Dependencies 
This project depends on the following: 
- ROS kinetic : To install follow ROS installation [page](http://wiki.ros.org/kinetic/Installation/Ubuntu) 
- catkin : To install catkin follow this [tutorial](http://wiki.ros.org/catkin?distro=indigo#Installing_catkin)


 and has been tested to work on ubuntu 16.04 LTS

# Build Instructions 
Assuming dependencies are met. Follow below commands:


```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone -b Week10_HW --single-branch https://github.com/saimouli/beginner_tutorials.git
cd ..
catkin_make
```

# Run Instructions 
Source the following command in bashrc

```
nano ~/.bashrc
source ~/catkin_ws/devel/setup.bash
source ~/.bashrc
```
To launch nodes, can either use a launch file or rosrun. Launch file will let user to launch several nodes in one command. Instructions for both types are below: 

## Using Launch File
To use launch file type the following command in the terminal:
```
roslaunch begineer_tutorials week10HW.launch
```
User can change the frequency at which the loop operates by the following command;
```
roslaunch begineer_tutorials week10HW.launch frequency:=7
```

## Using rosrun
In a new terminal[1] type 
```
roscore
```
In an another terminal[2] type
```
cd ~/catkin_ws
rosrun beginner_tutorials talker
```
- if you like to change the frequency enter:
- input must be an integer and greater than zero.
```
cd ~/catkin_ws
rosrun beginner_tutorials talker <frequency>
```

Yet in an another new terminal[3] type 
```
cd ~/catkin_ws
rosrun beginner_tutorials listener
```

# Service

If the user would like to change the output string message, type the following command in a new terminal
```
rosservice call /changeText "sample text"
```

# Logging
To see the message log in real time, use rqt_console GUI by typing the following command in a new terminal: 
```
rqt_console
```
