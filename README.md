# savi_ros_demo

Welcome to the savi_ros_demo repository. This is a demo project which is intended to show how to setup a project with the savi_ros_bdi package, available at https://github.com/NMAI-lab/savi_ros_bdi. For instructions on how to set up the savi_ros_bdi package please see the savi_ros_bdi github page.

## Overview
This repository contains scripts that demonstrate the functionality of the savi_ros_bdi package for ros. The savi_ros_bdi package listens for perceptions on the ```perceptions``` topic and publishes actions that are to be executed to the ```actions``` topic. Similarly, it listens to messages on the ```inbox``` topic and publishes messages to the ```outbox``` topic. This demo contains a complete demonstration project, including a Python script that generates example perceptions and messages and listens to to the requested actions and messages, which are printed to the terminal (see scripts/demo.py). Please note that this script is based on a tutorial on the ros website, available at http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29.

The repository also contains a simple AgentSpeak program, located at asl/demo.asl. This example BDI program implements the goal ```demonstrate```. This goal has to be passed as an 'achieve' message. Once it has that goal, it performs the action 'do(action)' when it receives a perception of the format 'time(1234)'. It also boradcasts the message heardTime(1234). Users who are not familiar with AgentSpeek are recommended to visit http://jason.sourceforge.net/ and read the Jason book.

Lastly, there is a resources directoy which contains settings.cfg as well as a bash script called configProject. The settings.cfg file contains the path to the AgentSpeak program as well as the name and type of the agent that is being programmed. The configProject script can be used to place the configProject file at the appropriate location in the savi_ros_bdi package.

## Configuration and Setup
These instructions assume that you already have a ros workspace with the savi_ros_bdi package set up, as per the instructions at that repository. This means that you have a ros workspace at ~/SAVI_ROS/rosjavaWorkspace which contains the savi_ros_bdi project, as described in the savi_ros_bdi Readme.

First, clone this repository to the src directory of your workspace.

```
$ git clone https://github.com/NMAI-lab/savi_ros_demo.git
```
Please note that to build this project from scratch, this could have been done using the following:
```
$ cd ~/SAVI_ROS/rosjavaWorkspace/src
$ catkin_create_pkg savi_ros_py std_msgs rospy roscpp
$ cd savi_ros_py 
$ mkdir scripts
$ mkdir asl
$ mkdir resources
```
The scripts folder holds the Python scripts used for publishing and subscribing to ros topics. The asl folder is the location of the AgentSpeak programs. Lastly, the resources folder contains settings.cfg, which needs to be copied to the savi_ros_bdi package for it to correctly configure the agent. There is also a bash script called configProject, which can be used for correctly moving this settings file to the correct location in the savi_ros_bdi package. To use this you must first update line 6 of this script with the correct directory location for the savi_ros_bdi package. Also, the settings.cfg file should be checked to confirm that the parameters are correct, most notably the location of the ASL file, the agent type and the agent name. This script can be run at the command line without parameters.
```
./configProject
```
In order to use the scripts, return to the project home directory and run catkin_make and source the setup.bash file.
```
$ cd ~/SAVI_ROS/rosjavaWorkspace
$ catkin_make
$ source devel/setup.bash
```

## Running
Before running the the demo scripts, roscore and savi_ros_bdi.Main need to be running. Please see the savi_ros_bdi Readme for instructions. It is then recommended that you run the listener first, followed by the talker. These will each need to be executed in thir own terminals.
```
$ rosrun savi_ros_demo demo.py
```
With these scripts running you will see details of the execution print to the terminals. Talker prints the messages being sent to ros, savi_ros_bdi will receive these messages and then publish actions to be executed to the actions topic. The listener prints these messages to the terminal.
