# Project 2 : Go Chase It
This repository is the project 2 of Udacity Robotics Software Engineer Nanodegree Program

![animation](https://user-images.githubusercontent.com/73100569/138605973-d4f12253-0b6e-4be0-829f-19d049c57b90.gif)

## Overview

In this project you'll create two ROS packages inside your `catkin_ws/src`: the `drive_bot` and the `ball_chaser` which will be used in Gazebo for all your upcoming projects in the [Udacity Robotics Software Engineer Nanodegree Program](https://www.udacity.com/course/robotics-software-engineer--nd209). Here are the steps to design the robot, house it inside your world, and program it to chase white-colored balls:  
1. `drive_bot`:  
   * Create a `my_robot` ROS package to hold your robot, the white ball, and the world.
   * Design a differential drive robot with the Unified Robot Description Format. Add two sensors to your robot: a lidar and a camera. Add Gazebo plugins for your robot’s differential drive, lidar, and camera. The robot you design should be significantly different from the one presented in the project lesson. Implement significant changes such as adjusting the color, wheel radius, and chassis dimensions. 
  
   * House your robot inside the world you built in the **Build My World** project.
   * Add a white-colored ball to your Gazebo world and save a new copy of this world.
   * The `world.launch` file should launch your world with the white-colored ball and your robot.
  
2. `ball_chaser`:
   * Create a `ball_chaser` ROS package to hold your C++ nodes.
   * Write a `drive_bot` C++ node that will provide a `ball_chaser/command_robot` service to drive the robot by controlling its linear x and angular z velocities. The service should publish to the wheel joints and return back the requested velocities.
   * Write a `process_image` C++ node that reads your robot’s camera image, analyzes it to determine the presence and position of a white ball. If a white ball exists in the image, your node should request a service via a client to drive the robot towards it.
   * The `ball_chaser.launch` should run both the `drive_bot` and the `process_image` nodes.  

## Build Dependencies  
* Gazebo >= 7.0  
* ROS Kinetic/Melodic  
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Structure
```
.P2_Go_Chase_It                        # Go Chase It Project
    ├── my_robot                       # my_robot package                   
    │   ├── launch                     # launch folder for launch files   
    │   │   ├── robot_description.launch
    │   │   ├── world.launch
    │   ├── meshes                     # meshes folder for sensors
    │   │   ├── hokuyo.dae
    │   ├── rviz                       # custom rviz configuration files
    │   │   ├── rviz_config.rviz
    │   ├── urdf                       # urdf folder for xarco files
    │   │   ├── my_robot.gazebo
    │   │   ├── my_robot.xacro
    │   ├── worlds                     # world folder for world files
    │   │   ├── seokhun.world
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info
    ├── ball_chaser                    # ball_chaser package                   
    │   ├── launch                     # launch folder for launch files   
    │   │   ├── ball_chaser.launch
    │   ├── src                        # source folder for C++ scripts
    │   │   ├── drive_bot.cpp
    │   │   ├── process_image.cpp
    │   ├── srv                        # service folder for ROS services
    │   │   ├── DriveToTarget.srv
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info                  
    └──
```

## Build and Run Instructions
1. Make sure you have installed `Build Dependencies`:
2. Clone this repository as below structure:
  ```bash
  catkin_ws                              # your catkin workspace
      ├── src                             
      │   ├── my_robot                       # my_robot package                   
      │   └── ball_chaser                    # ball_chaser package           
      └── ...
  ```
3. Compile it with:
   ```bash
   $ cd /home/workspace/catkin_ws/
   $ catkin_make
   ```
4. Launch `my_robot` in Gazebo to load both the world and plugins with rviz
   ```bash
   $ cd /home/workspace/catkin_ws/
   $ source devel/setup.bash
   $ roslaunch my_robot world.launch
   ```
5. Launch `ball_chaser` and `process_image` nodes
   ```bash
   $ cd /home/workspace/catkin_ws/
   source devel/setup.bash
   roslaunch ball_chaser ball_chaser.launch
   ```
