# ENPM808X - ROS Beginner Tutorial
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

# Overview
This repository contains basic ROS C++ subscriber and publisher taken from the ROS [website](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29). 

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
git clone --recursive https://github.com/saimouli/beginner_tutorials.git
cd ..
catkin_make
```

# Run Instructions 
Source the following command in bashrc

```
nano ~/.bashrc
source devel/setup.bash
source ~/.bashrc
```

In a new terminal[1] type 
```
roscore
```
In an another terminal[2] type
```
cd ~/catkin_ws
rosrun beginner_tutorials talker
```

Yet in an another new terminal[3] type 
```
cd ~/catkin_ws
rosrun beginner_tutorials listener
```
