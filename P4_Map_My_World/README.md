# Project 4 : Map My World
This repository is the project 4 of Udacity Robotics Software Engineer Nanodegree Program

![full](https://user-images.githubusercontent.com/73100569/141305682-ec32555f-17bb-4ece-b730-4f471ca0b594.png)

![feature_matching](https://user-images.githubusercontent.com/73100569/141305674-6acab41a-945c-48f3-a153-bbddcf3ecffb.png)

## Overview

In this project you will create a 2D occupancy grid and 3D octomap from a simulated environment using your own robot with the RTAB-Map package.  
RTAB-Map (Real-Time Appearance-Based Mapping) is a popular solution for SLAM to develop robots that can map environments in 3D. RTAB-Map has good speed and memory management, and it provides custom developed tools for information analysis. Most importantly, the quality of the documentation on ROS Wiki (http://wiki.ros.org/rtabmap_ros) is very high. Being able to leverage RTAB-Map with your own robots will lead to a solid foundation for mapping and localization well beyond this Nanodegree program.  

For this project we will be using the `rtabmap_ros` package, which is a ROS wrapper (API) for interacting with RTAB-Map. Keep this in mind when looking at the relative documentation.  
* You will develop your own package to interface with the rtabmap_ros package.  
* You will build upon your localization project to make the necessary changes to interface the robot with RTAB-Map. An example of this is the addition of an RGB-D camera.  
* You will ensure that all files are in the appropriate places, all links are properly connected, naming is properly setup and topics are correctly mapped. Furthermore you will need to generate the appropriate launch files to launch the robot and map its surrounding environment.  
* When your robot is launched you will teleop around the room to generate a proper map of the environment.  

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
* ROS rtabmap-ros package  
   ```bash
   # if kinetic,
   $ sudo apt-get install ros-kinetic-rtabmap-ros
   # if melodic,
   $ sudo apt-get install ros-melodic-rtabmap-ros
   ```

## Structure
```bash
.P4_Map_My_World                                     # Where Am I Project
    ├── my_robot                                   # my_robot package                  
    │   ├── config                                 # config folder for configuration files of move_base package
    │   │   ├── __MACOSX
    │   │   ├── base_local_planner_params.yaml
    │   │   ├── costmap_common_params.yaml
    │   │   ├── global_costmap_params.yaml
    │   │   └── local_costmap_params.yaml 
    │   ├── db_dir                                 #### You can download the database I already generated 
    │   │                                          #### in this project at google drive(see below). 
    │   ├── launch                                 # launch folder for launch files   
    │   │   ├── amcl.launch
    │   │   ├── robot_description.launch
    │   │   └── world.launch
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
    ├── teleop_twist_keyboard                      # Generic Keyboard Teleop for ROS
    │   ├── CHANGELOG.rst
    │   ├── CMakeLists.txt
    │   ├── README.md
    │   ├── package.xml
    │   └── teleop_twist_keyboard.py
```

## Download Database
You can download the database I already generated in this project at google drive [here](https://drive.google.com/file/d/1OfzN0tMGXHmWJVkqjovUTaXFkU6kXeJ8/view?usp=sharing).

## Build and Run Instructions
1. Make sure you have installed `Build Dependencies`:
   
2. Clone this repository as below structure:
   ```bash
   catkin_ws                                 # your catkin workspace
      ├── src                         
      │   ├── my_robot                       # my_robot package             
      │   ├── teleop_twist_keyboard          # teleop_twist_keyboard package          
      │   ├── ...     
      └── ...
   ```

3. Compile it with:
   ```bash
   $ cd /home/workspace/catkin_ws/
   $ catkin_make
   ```

4. Launch `my_robot` in Gazebo to load both the world and plugins with rviz at one terminal.
   ```bash
   $ cd /home/workspace/catkin_ws/
   $ source devel/setup.bash
   $ roslaunch my_robot world.launch
   ```

5. Launch `rtabmap-ros` package by launching the `mapping.launch` file at another terminal.
   ```bash
   $ cd /home/workspace/catkin_ws/
   $ source devel/setup.bash
   $ roslaunch my_robot mapping.launch
   ```

6. Run the `teleop` node at the other terminal.
   ```bash
   $ cd /home/workspace/catkin_ws/
   $ source devel/setup.bash
   $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
   ```
   - You can navigate the robot around the environment to scan its surrounding by following the instructions at the terminal. 
   - The `rtabmap-ros` package will save the resulted map with the localized trajectory of the robot in `db_dir/rtabmap.db`
   - Remember to rename your `db_dir/rtabmap.db` before moving on to map a new environment,  since it will be deleted if you relaunch `mapping.launch`

7. Open new terminal, and open the database file using `rtabmap-databaseViewer` as belows:
   ```bash
   $ cd /home/workspace/catkin_ws/src/my_robot
   $ rtabmap-databaseViewer db_dir/rtabmap.db
   ```
   - You will need to add some windows to get a better view of the relevant information, so:
     - Say yes to using the database parameters if it pops up.
     - View -> Constraint View
     - View -> Graph View
     
