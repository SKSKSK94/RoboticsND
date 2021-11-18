#!/bin/sh
export TURTLEBOT3_MODEL=waffle
export TURTLEBOT3_GAZEBO_WORLD_FILE=$(pwd)/src/RoboticsND/P5_Home_Service_Robot/worlds/home_service.world

xterm  -e  " source devel/setup.bash; roslaunch spawn_home_service_world turtlebot3_home_service_world.launch " &
sleep 3
xterm  -e  " source devel/setup.bash; rosrun gmapping slam_gmapping " &
sleep 5
xterm  -e  " source devel/setup.bash; roslaunch spawn_home_service_world turtlebot3_gazebo_rviz.launch " & 
sleep 3
xterm  -e  " source devel/setup.bash; roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch " 