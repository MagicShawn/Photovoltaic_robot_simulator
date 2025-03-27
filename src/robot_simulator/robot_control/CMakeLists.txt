cmake_minimum_required(VERSION 3.0.2)
project(robot_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(velocity_controller src/velocity_controller.cpp)
target_link_libraries(velocity_controller ${catkin_LIBRARIES})