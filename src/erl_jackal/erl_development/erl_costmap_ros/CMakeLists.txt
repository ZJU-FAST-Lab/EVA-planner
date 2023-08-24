cmake_minimum_required(VERSION 3.1)
project(erl_costmap_ros)

# Set Default build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations")


# Add External Dependencies
find_package(PkgConfig)
find_package(catkin_simple REQUIRED)

# Boost
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread system iostreams filesystem program_options date_time timer chrono serialization)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(include ${EIGEN3_INCLUDE_DIR})

# OpenCV
FIND_PACKAGE(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

catkin_simple()

# Set link libraries
LIST(APPEND LINK_LIBS
        ${catkin_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES})

add_executable(erl_costmap_ros_node src/erl_costmap_ros.cpp)
target_link_libraries(erl_costmap_ros_node ${LINK_LIBS})

cs_install()
cs_export()

