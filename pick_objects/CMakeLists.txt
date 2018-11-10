cmake_minimum_required(VERSION 2.8.3)
project(pick_objects)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
)
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_navigation_goals src/simple_navigation.cpp)
target_link_libraries(simple_navigation_goals ${catkin_LIBRARIES})
add_dependencies(
  simple_navigation_goals
  ${pick_objects_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

