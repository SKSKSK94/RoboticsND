# Project 1 : Build My World
This repository is the project 1 of Udacity Robotics Software Engineer Nanodegree Program
![image](https://user-images.githubusercontent.com/73100569/137704090-e30dbb05-466e-492c-b8d6-d8e052aa9b88.png)

## Overview
In this project you'll create your simulation world in Gazebo for all your upcoming projects in the [Udacity Robotics Software Engineer Nanodegree Program](https://www.udacity.com/course/robotics-software-engineer--nd209).

1. Build a single floor wall structure using the Building Editor tool in Gazebo. Apply at least one feature, one color, and optionally one texture to your structure. Make sure there's enough space between the walls for a robot to navigate.
2. Model any object of your choice using the Model Editor tool in Gazebo. Your model links should be connected with joints.
3. Import your structure and two instances of your model inside an empty Gazebo World.
4. Import at least one model from the Gazebo online library and implement it in your existing Gazebo world.
5. Write a C++ World Plugin to interact with your world. Your code should display “Welcome to ’s World!” message as soon as you launch the Gazebo world file.

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
.P1_Build_My_World                 # Build My World Project 
├── model                          # Model files 
│   ├── office
│   │   ├── model.config
│   │   ├── model.sdf
│   ├── robot
│   │   ├── model.config
│   │   ├── model.sdf
├── script                         # Plugin files 
│   ├── welcome.cpp
├── world                          # World files
│   ├── office_world.world
├── CMakeLists.txt
```

## Build Instructions
1. Make sure you have installed `Build Dependencies`:
2. Clone this repository
3. Go to the project folder `P1_Build_My_World`
4. Create a build folder `mkdir build && cd build`
5. Build with cmake `cmake .. && make`
6. Add the build folder to the Gazebo plugin path: 
```bash
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:<path_to_build_folder>
# For example as belows : 
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/home/RoboticsND/P1_Build_My_World/build
```
7. Go back to the project folder `P1_Build_My_World`
8. Launch Gazebo world by `gazebo world/office_world`
