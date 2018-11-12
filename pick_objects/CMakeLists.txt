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

add_executable(pick_objects src/pick_objects.cpp)
target_link_libraries(pick_objects ${catkin_LIBRARIES})
add_dependencies(
  pick_objects
  ${pick_objects_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

