#!/bin/sh

xterm -e " roslaunch turtlebot_simulator/turtlebot_gazebo/launch/turtlebot_world.launch world_file:=/home/kjames/catkin_ws/src/World/corridor_l " &
sleep 5
xterm -e " roslaunch turtlebot_interactions/turtlebot_rviz_launchers/launch/view_navigation.launch " &
sleep 5
xterm -e " roslaunch turtlebot_simulator/turtlebot_gazebo/launch/amcl_demo.launch " &
sleep 5
xterm -e " rosrun pick_objects pick_objects " &
sleep 5
