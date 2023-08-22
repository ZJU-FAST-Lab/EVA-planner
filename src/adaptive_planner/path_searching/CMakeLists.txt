cmake_minimum_required(VERSION 3.0.2)
project(path_searching)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(NLOPT REQUIRED nlopt)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env
  cv_bridge
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES path_searching
 CATKIN_DEPENDS plan_env
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")
#set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -Wall -g")

add_library( path_searching
        src/astar3d.cpp
        src/low_mpc_optimizer.cpp
        src/high_mpcc_optimizer.cpp
    )
#target_link_libraries( path_searching
#    ${catkin_LIBRARIES}
#    /usr/local/lib/libnlopt.so
#    )


target_link_libraries( path_searching
    ${catkin_LIBRARIES}
    nlopt
    )
