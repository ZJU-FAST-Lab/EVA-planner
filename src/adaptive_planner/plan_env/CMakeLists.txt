cmake_minimum_required(VERSION 3.0)
project(plan_env)

find_package(OpenCV REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  message_filters
  erl_costmap_ros
#  erl_conversions
)

# find_package(Eigen3 REQUIRED)

# if (NOT Eigen3_INCLUDE_DIRS)
#     get_target_property(Eigen3_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
# endif ()

# Eigen3
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})


find_package(PCL REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES plan_env
#    CATKIN_DEPENDS roscpp std_msgs visualization_msgs cv_bridge message_filters erl_costmap_ros erl_conversions
        CATKIN_DEPENDS roscpp std_msgs visualization_msgs cv_bridge message_filters erl_costmap_ros
    DEPENDS Eigen3
)

message(WARNING ${catkin_INCLUDE_DIRS})

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O3 -Wall")

add_library( plan_env
    src/sdf_map.cpp
    src/edt_environment.cpp
#    src/raycast.cpp
    )
#add_executable( sdf_map
#        src/sdf_map.cpp
#        src/edt_environment.cpp
#        src/raycast.cpp
#        )
target_link_libraries( plan_env
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    )
message(STATUS ${PROJECT_NAME} DONE)
