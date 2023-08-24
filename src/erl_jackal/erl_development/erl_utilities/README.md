# The erl_utilities package
This package contains various utility functions for Geometry, Mapping, Polynomial solving, Timing, and Random Number generation (RNG). These functions are header-only, and can easily be included in other projects.

## Build Instructions
#### Dependencies:
  - `Eigen`: apt install libeigen3-dev
  - `PCL`: apt install libpcl-dev
  - `YAML-CPP`: apt install libyaml-cpp-dev

or simply run following commands:
    
    sudo apt-get update
    sudo apt install -y libeigen3-dev libpcl-dev libyaml-cpp-dev libproj-dev cmake



#### A) Compilation via catkin (Default)

    mv erl_utilities ~/catkin_ws/src
    git clone git@github.com:catkin/catkin_simple.git ~/catkin_ws/src
    cd ~/catkin_ws && catkin_make
    
#### B) Compilation via cmake

    mkdir build
    cd build
    cmake .. -DUSE_ROS=OFF
    make
    make test
    


## Installation
### With Catkin (Default)
To include these utilities in other projects, the easiest way is to use Catkin.

In an existing Catkin workspace, clone this repository into the `src` folder.

Then, add the line `  <depend>erl_utilities</depend>` to your project's `package.xml` file.

Using `catkin_simple`, add:

    
    # Catkin Simple
    FIND_PACKAGE(catkin_simple REQUIRED)
    catkin_simple()

to your CMakeLists.txt file , which adds all dependencies from the `package.xml` file.

Then the appropriate headers can be included via: `#include<erl_utilities/XXX.h`

### With CMake
To include this package in your existing CMake project, clone this package into a directory DIR.

Then in your CMakeLists.txt:

    `include_directories(DIR/erl_utilities/include)`

It is possible to configure your project to use this as a dependency that can be downloaded at build time via Git submodules. See
the project: [erl_astar](https://bitbucket.org/ExistentialRobotics/erl_astar)

