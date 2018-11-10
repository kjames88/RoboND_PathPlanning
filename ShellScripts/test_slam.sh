#!/bin/sh

xterm -e " roslaunch turtlebot_simulator/turtlebot_gazebo/launch/turtlebot_world.launch " &
sleep 5
xterm -e " roslaunch turtlebot_simulator/turtlebot_gazebo/launch/gmapping_demo.launch " &
sleep 5
xterm -e " roslaunch turtlebot_interactions/turtlebot_rviz_launchers/launch/view_navigation.launch " &
sleep 5
xterm -e " roslaunch turtlebot/turtlebot_teleop/launch/keyboard_teleop.launch " &
sleep 5

