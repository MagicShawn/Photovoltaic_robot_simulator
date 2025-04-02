#CMake的版本号
cmake_minimum_required(VERSION 2.8.3)
#项目名称
project(astar)
#C++编译版本
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14)

set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

#找到编译需要的其他CMake/Catkin package
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
#
catkin_package(
    INCLUDE_DIRS include
)
#
include_directories(
    include
    SYSTEM
    third_party
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")
#可执行文件
add_executable(map_gen
    src/2d_map_gen.cpp
)

target_link_libraries(map_gen
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

# add_executable(my_astar
#     src/my_Astar.cpp
# )

# target_link_libraries(my_astar
#     ${catkin_LIBRARIES}
#     ${PCL_LIBRARIES}
# )

add_executable(my_map
    src/my_map.cpp
)

target_link_libraries(my_map
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(solarpanel_map
    src/solarpanel_map.cpp
)

target_link_libraries(solarpanel_map
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(master
    src/visualize.cpp
    src/master.cpp
    src/custom_Astar.cpp
)

target_link_libraries(master
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(custom_master
    src/visualize.cpp
    src/custom_master.cpp
    src/custom_Astar.cpp
)

target_link_libraries(custom_master
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)