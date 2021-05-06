cmake_minimum_required(VERSION 3.0.2)
project(plan_manage)

set(CMAKE_VERBOSE_MAKEFILE "false")
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  nav_msgs
  plan_env
  path_searching
  quadrotor_msgs
  cv_bridge
  visualization_msgs
  traj_utils
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_manage
 CATKIN_DEPENDS plan_env path_searching traj_utils message_runtime
#  DEPENDS system_lib
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

include_directories(
    include
    SYSTEM 
    third_party
  	${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  	${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS} 
)

add_executable(adaptive_planner_node
	src/adaptive_planner_node.cpp
  src/adaptive_replan_fsm.cpp
  src/planner_manager.cpp
)

target_link_libraries(adaptive_planner_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)

