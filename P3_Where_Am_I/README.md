# Project 3 : Where Am I?
This repository is the project 3 of Udacity Robotics Software Engineer Nanodegree Program

![localization](https://user-images.githubusercontent.com/73100569/139944066-eecb88d1-77ce-4c21-bb00-ab35a7d548bb.gif)

## Overview

In this project you'll utilize ROS AMCL (Adaptive Monte Carlo Localization) package to accurately localize a mobile robot inside a map in the Gazebo simulation environments. Here are the steps to learn several aspects of robotic software engineering with a focus on ROS:  
* Create a ROS package that launches a custom robot model in a custom Gazebo world  
* Utilize the ROS AMCL package and the Tele-Operation / Navigation Stack to localize the robot  
* Explore, add, and tune specific parameters corresponding to each package to achieve the best possible localization results  

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
  * 
* ROS navigation package  
   ```bash
   # if kinetic,
   $ sudo apt-get install ros-kinetic-navigation
   # if melodic,
   $ sudo apt-get install ros-melodic-navigation
   ```
* ROS map_server package  
   ```bash
   # if kinetic,
   $ sudo apt-get install ros-kinetic-map-server
   # if melodic,
   $ sudo apt-get install ros-melodic-map-server
   ```
* ROS move_base package  
   ```bash
   # if kinetic,
   $ sudo apt-get install ros-kinetic-move-base
   # if melodic,
   $ sudo apt-get install ros-melodic-move-base
   ```
* ROS amcl package  
   ```bash
   # if kinetic,
   $ sudo apt-get install ros-kinetic-amcl
   # if melodic,
   $ sudo apt-get install ros-melodic-amcl
   ```

## Structure
```bash
.P3_Where_Am_I                                     # Where Am I Project
    ├── my_robot                                   # my_robot package                  
    │   ├── config                                 # config folder for configuration files of move_base package
    │   │   ├── __MACOSX
    │   │   ├── base_local_planner_params.yaml
    │   │   ├── costmap_common_params.yaml
    │   │   ├── global_costmap_params.yaml
    │   │   └── local_costmap_params.yaml 
    │   ├── launch                                 # launch folder for launch files   
    │   │   ├── amcl.launch
    │   │   ├── robot_description.launch
    │   │   └── world.launch
    │   ├── maps                                   # maps folder for maps  
    │   │   ├── seokhun.pgm
    │   │   └── seokhun.yaml
    │   ├── meshes                                 # meshes folder for sensors
    │   │   └── hokuyo.dae
    │   ├── rviz                                   # custom rviz configuration files
    │   │   └── rviz_config.rviz
    │   ├── urdf                                   # urdf folder for xarco files
    │   │   ├── my_robot.gazebo
    │   │   └── my_robot.xacro
    │   ├── worlds                                 # world folder for world files
    │   │   └── seokhun.world
    │   ├── CMakeLists.txt                         # compiler instructions
    │   └── package.xml                            # package info
    ├── pgm_map_creator                            # Package for creating pgm map from Gazebo world file for ROS localization
    │   ├── CMakeLists.txt
    │   ├── CODEOWNERS
    │   ├── LICENSE
    │   ├── README.md
    │   ├── launch
    │   │   └── request_publisher.launch
    │   ├── maps
    │   ├── msgs
    │   │   ├── CMakeLists.txt
    │   │   └── collision_map_request.proto
    │   ├── package.xml
    │   ├── src
    │   │   ├── collision_map_creator.cc
    │   │   └── request_publisher.cc
    │   └── world
    │   │   └── seokhun.world
    ├── teleop_twist_keyboard                      # Generic Keyboard Teleop for ROS
    │   ├── CHANGELOG.rst
    │   ├── CMakeLists.txt
    │   ├── README.md
    │   ├── package.xml
    │   └── teleop_twist_keyboard.py
```

## Build and Run Instructions
1. Make sure you have installed `Build Dependencies`:
2. Clone this repository as below structure:
   ```bash
   catkin_ws                                 # your catkin workspace
      ├── src                         
      │   ├── my_robot                       # my_robot package                   
      │   ├── pgm_map_creator                # pgm_map_creator package      
      │   ├── teleop_twist_keyboard          # teleop_twist_keyboard package          
      │   ├── ...     
      └── ...
   ```
3. Compile it with:
   ```bash
   $ cd /home/workspace/catkin_ws/
   $ catkin_make
   ```
4. Launch `map_server`, `amcl` and `move_base` nodes at one terminal.
   ```bash
   $ cd /home/workspace/catkin_ws/
   $ source devel/setup.bash
   $ roslaunch my_robot amcl.launch
   ```
5. Launch `my_robot` in Gazebo to load both the world and plugins with rviz at another terminal.
   ```bash
   $ cd /home/workspace/catkin_ws/
   $ source devel/setup.bash
   $ roslaunch my_robot world.launch
   ```
6. Testing
   You have two options to control your robot while it localize itself here:
      1. Send navigation goal via RViz
         Your first option would be sending a `2D Nav Goal` from RViz. The `move_base` will try to navigate your robot based on the localization. Based on the new observation and the odometry, the robot to further perform the localization.  
         Click the `2D Nav Goal` button in the toolbar, then click and drag on the map to send the goal to the robot. It will start moving and localize itself in the process. If you would like to give `amcl` node a nudge, you could give the robot an initial position estimate on the map using `2D Pose Estimate`.  
      2. Send move command via teleop package.
         You could also use teleop node to control your robot and observe it localize itself in the environment.  
         Open another terminal and launch the `teleop` script:  
         ```bash
         $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
         ```
         You could control your robot by keyboard commands now.
