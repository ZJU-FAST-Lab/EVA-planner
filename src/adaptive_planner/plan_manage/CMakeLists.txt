cmake_minimum_required(VERSION 3.0.2)
project(plan_manage)

set(CMAKE_VERBOSE_MAKEFILE "false")
#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
#CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#if(COMPILER_SUPPORTS_CXX14)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
#elseif(COMPILER_SUPPORTS_CXX0X)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#else()
#    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
#endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ADDITIONAL_CXX_FLAG "-Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")

# find_package(Eigen3 3.4.90 CONFIG REQUIRED)
find_package(PCL 1.7 REQUIRED)


FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})


find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  nav_msgs
  quadrotor_msgs
  plan_env
  path_searching
  cv_bridge
  traj_utils
  visualization_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_manage
 CATKIN_DEPENDS plan_env path_searching traj_utils message_runtime
#  DEPENDS system_lib
)

# if (NOT Eigen3_INCLUDE_DIRS)
#     get_target_property(Eigen3_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
# endif ()



find_package(PCL REQUIRED)

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

