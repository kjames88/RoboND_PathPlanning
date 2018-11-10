#!/bin/sh

xterm -e " roslaunch turtlebot_simulator/turtlebot_gazebo/launch/turtlebot_world.launch world_file:=/home/kjames/catkin_ws/src/World/interior_world_2 " &
sleep 5
xterm -e " roslaunch turtlebot_simulator/turtlebot_gazebo/launch/gmapping_demo.launch " &
sleep 5
xterm -e " roslaunch turtlebot_interactions/turtlebot_rviz_launchers/launch/view_navigation.launch " &
sleep 5
xterm -e " rosrun wall_follower wall_follower_node cmd_vel:=/cmd_vel_mux/input/teleop | tee /tmp/out " &
sleep 5

