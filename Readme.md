# RoboND-PathPlanning

12 November 2018

## Environment

A simple interior environment was constructed in the Gazebo Building Editor.  The purpose of the project is to map the environment using slam\_gmapping in ROS, then to navigate to two locations on the map.

## Wall Follower

Mapping is conducted by an autonomous wall following robot.  The continuous wall perimeter of the environment enables the robot to travel repeatedly around the interior; it does not follow the exerior perimeter successfully.  Attempts to modify the wall follower for tighter tracking around outside corners resulted in oscillations or other stuck conditions, whereas the default behavior does not fill in the wall obstruction along shallow indents.  To solve this problem, the wall follower was modified to reverse its direction every several minutes.  For example, the default behavior is to keep the wall on the robot's right side, and this was changed to the left side.  Approaching in the opposite direction enables the robot's sensors to sweep walls that fall behind the robot in the forward direction (in particular around outside corners).  Without this step there was the risk that the robot could try to navigate off the map due to a missing wall.

The robot essentially steers away from the wall when it gets close, and steers toward the wall when it gets too far away.  Otherwise it goes straight.  Searching in open space is an adaptation of getting too far away from the wall:  turning right (by default) should eventually lead to another wall segment to follow.

## Goals

Goals are hard-coded in pick\_object.cpp and add\_markers.cpp.  As presented, there are only two goals, the first of which is the _pickup_ location, and the second is the _dropoff_ location.  A marker is drawn for each location.  Initially, the markers were set to display until the robot arrived at a location, then erase, wait 5 seconds, and appear at the next location.  However, following the project specification, a marker is now drawn at the first location, erased when the robot arrives, and then drawn at the second location once the robot arrives.

