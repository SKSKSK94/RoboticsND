#!/bin/sh
export TURTLEBOT3_MODEL=waffle
export TURTLEBOT3_GAZEBO_WORLD_FILE=$(pwd)/src/P5_Home_Service_Robot/worlds/home_service.world
map_file=$(pwd)/src/P5_Home_Service_Robot/maps/home_service.yaml

xterm  -e  " source devel/setup.bash; roslaunch turtlebot3_gazebo turtlebot3_home_service_world.launch " &
sleep 3
xterm  -e  " source devel/setup.bash; roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=${map_file} "