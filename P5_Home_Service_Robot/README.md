# Project 5 : Home Service Robot
This repository is the project 5 of Udacity Robotics Software Engineer Nanodegree Program


Home Service Robot Simulation             |
:-------------------------:|
![hs_project_](https://user-images.githubusercontent.com/73100569/142504156-7fdca566-dd38-4f9a-ac1f-f49dd03f9066.gif)|

## Overview

In this project, you will use everything you learned in the Nanodegree Program to build a Home Service Robot in ROS.

#### Mapping

- You will create a `test_slam.sh` script file and launch it to manually test SLAM.
- A functional map of the environment should be created which would be used for localization and navigation tasks.
  
#### Localization and Navigation

- You will create a `test_navigation.sh` script file to launch it for manual navigation test.
  - Your robot should be able to navigate in the environment after a 2D Nav Goal command is issued.
- You will create a `pick_objects.sh` file that will send multiple goals for the robot to reach.
   - Your robot travels to the desired pickup zone, displays a message that it reached its destination, waits 5 seconds, travels to the desired drop off zone, and displays a message that it reached the drop off zone.

#### Home Service Functions

- You will create a `add_marker.sh` file that will publish a marker to rviz.
   - The marker should initially be published at the pickup zone. 
   - After 5 seconds it should be hidden. 
   - Then after another 5 seconds it should appear at the drop off zone. 
   - You should write a `home_service.sh` file that will run all the nodes in this project.
  
- Your home service robot should be simulated as follow:
  - Initially show the marker at the pickup zone
  - Hide the marker once your robot reaches the pickup zone
  - Wait 5 seconds to simulate a pickup
  - Show the marker at the drop off zone once your robot reaches it

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
```bash
.P5_Home_Service_Robot                             # Where Am I Project
    ├── add_markers                                # add_markers package                  
    │   ├── launch                                 # launch folder for launch files
    │   │   ├── add_markers_test.launch
    │   │   └── add_markers.launch
    │   ├── src                                 
    │   │   ├── add_markers_test.cpp               # source code for add_markers_test node
    │   │   └── add_markers.cpp                    # source code for add_markers node
    │   ├── CMakeLists.txt                         # compiler instructions
    │   └── package.xml                            # package info
    ├── maps                                       # simulation map file                  
    │   ├── home_service.pgm                          
    │   └── home_service.yaml                      # package info
    ├── params                                     # pickup and dropoff zone parameters        
    │   └── goal.yaml                      
    ├── pick_objects                               # pick_objects package                  
    │   ├── launch                                 # launch folder for launch files
    │   │   └── pick_objects.launch
    │   ├── src                                 
    │   │   └── pick_objects.cpp                   # source code for pick_objects node
    │   ├── CMakeLists.txt                         # compiler instructions
    │   └── package.xml                            # package info
    ├── rvizConfig                                 # configuration directory for rviz                  
    │   ├── rviz_with_marker.rviz
    │   └── test_slam_rviz.rviz
    ├── scripts                                    # shell scripts files             
    │   ├── add_marker.sh                          # shell script to model virtual objects in rviz
    │   ├── home_service.sh                        # shell script to launch home service robot 
    │   ├── pick_objects.sh                        # shell script to send multiple goals  
    │   ├── test_navigation.sh                     # shell script to test localization and navigation
    │   └── test_slam.sh                           # shell script to test SLAM
    ├── slam_gmapping                              # slam gmapping package
    ├── spawn_home_service_world                   # spawn home service world package
    ├── turtlebot3                                 # turtlebot3 package
    ├── worlds                                     # world files for simulation
    │   └── home_service.world
```


## Build and Run Instructions   
1. Clone this repository as below structure:
   ```bash
   catkin_ws                                 # your catkin workspace
      ├── src                         
      │   ├── RoboticsND   
      │   │   ├── P5_Home_Service_Robot           
      │   │   │   ├── add_markers                            
      │   │   │   ├── maps                                 
      │   │   │   ├── params                            
      │   │   │   ├── pick_objects                            
      │   │   │   ├── rvizConfig                            
      │   │   │   ├── scripts                            
      │   │   │   ├── slam_gmapping                      
      │   │   │   ├── spawn_home_service_world                            
      │   │   │   ├── turtlebot3                                          
      │   │   │   ├── worlds         
      │   │   │   ├── ...     
      └─ ...
   ```

2. Make sure you have installed `Build Dependencies`: 
   - `turtlebot3_simulations` 
      ```bash
      $ cd /home/workspace/catkin_ws/src/RoboticsND/P5_Home_Service_Robot

      # for kinetic
      $ git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

      # for melodic
      $ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
      ```
   - `turtlebot3_msgs`
      ```bash
      $ cd /home/workspace/catkin_ws/src/RoboticsND/P5_Home_Service_Robot
      $ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs
      ```
   - `rospkg`
      ```bash
      $ pip install rospkg
      ```

3. Compile it with:
   ```bash
   $ cd /home/workspace/catkin_ws/
   $ catkin_make
   ```

4. For manually testing SLAM:
   ```bash
   $ cd /home/workspace/catkin_ws/
   $ src/RoboticsND/P5_Home_Service_Robot/scripts/test_slam.sh 
   ```

5. For starting the simulation world with turtlebot3, movebase and teleop:
   ```bash
   $ cd /home/workspace/catkin_ws/
   $ src/RoboticsND/P5_Home_Service_Robot/scripts/test_navigation.sh 
   ```

6. For autonomously commanding the turtlebot3 to travel to both desired pickup and drop off zones:
   ```bash
   $ cd /home/workspace/catkin_ws/
   $ src/RoboticsND/P5_Home_Service_Robot/scripts/pick_objects.sh 
   ```

7. For creating a virtual object in rviz:
   ```bash
   $ cd /home/workspace/catkin_ws/
   $ src/RoboticsND/P5_Home_Service_Robot/scripts/add_markers.sh
   ```

8. For complete simulation of home service robot:
   ```bash
   $ cd /home/workspace/catkin_ws/
   $ src/RoboticsND/P5_Home_Service_Robot/scripts/home_service.sh
   ```