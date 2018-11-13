# ENPM808X - ROS Beginner Tutorial, TF, Unit Testing, Bag files
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

# Overview
This repository contains basic ROS C++ subscriber and publisher taken from the ROS [website](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29). 
- Takler (src/talker.cpp): publisher
- Listener (src/listener.cpp): subscriber
- TalkerTest (test/talkerTest.cpp): unit test 

A launch file named ```week10HW.launch``` can be used to launch both talker, listener, and record messages in .bag format 

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
git clone -b Week11_HW --single-branch https://github.com/saimouli/beginner_tutorials.git
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
roslaunch beginner_tutorials week10HW.launch
```
User can change the frequency at which the loop operates by the following command;
```
roslaunch beginner_tutorials week10HW.launch frequency:=7
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

# Inspecting TF Frames
The talker.cpp publishes /tf topic of a non zero static tf frames called /talk with respect to the /world frame. 
 
 
The translation of tf broadcaster is composed in polar coordinates using ros::time. This will make sure for certain range of time stamps there exists a unique values. To visualize the topics being produced type the following in a new terminal: 
 
 ```
 cd ~/catkin_ws
 rosrun rqt_tf_tree rqt_tf_tree
 ```
To echo the values type the following in a new terminal: 
 ```
 cd ~/catkin_ws
 rosrun tf tf_echo /world /talk
 ```
 
View_frames will produce a diagram of the broadcaster frame. While running the demo type the following in the new terminal
 ```
 cd ~/catkin_ws
 rosrun tf view_frames
 ```
Above command will produce a pdf file which can be viewed in the catkin workspace. An example of this pdf can be viewed in the results folder
 
# Running ROS Unit Tests
To run the ros unit testing type the following in a new terminal 

## Using Launch File 
```
cd ~/catkin_ws
rostest beginner_tutorials talkerTest.launch
```
## Using catkin_make
```
cd ~/catkin_ws
catkin_make run_tests beginner_tutorials
```
sample output
```
... logging to /home/viki/.ros/log/rostest-ubuntu-5582.log
[ROSUNIT] Outputting test results to /home/viki/catkin_ws/build/test_results/beginner_tutorials/rostest-test_talkerTest.xml
testtalkerTest ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-talkerTest/testServiceExsistance][passed]
[beginner_tutorials.rosunit-talkerTest/testServiceMessageUpdate][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/viki/.ros/log/rostest-ubuntu-5582.log
-- run_tests.py: verify result "/home/viki/catkin_ws/build/test_results/beginner_tutorials/rostest-test_talkerTest.xml"
```

# Playing bag files 
A recorded ros bag file is located in the results folder. To play the ros bag file type the following commands: 

In a new terminal 
```
roscore
```

Open another new terminal
```
cd ~/catkin_ws
rosrun beginner_tutorials listener
```

In an another new terminal
```
cd ~/catkin_ws/src/beginner_tutorials/results
rosbag play rostopicsRecord.bag
```
The /chatter messages that have been recorded can be viewed in the listner node 

# Recording bag files with launch file 
```
roslaunch beginner_tutorials week10HW.launch rosbagEnable:=true
```

