# EVA-planner

**EVA**-planner: an **E**n**V**ironmental **A**daptive Gradient-based Local Planner for Quadrotors.

## 0. Overview
**Author**: Lun Quan, Zhiwei Zhang, Xingguang Zhong, Chao Xu and [Fei Gao](https://ustfei.com/) from [ZJU FAST Lab](http://zju-fast.com/).

**Related Paper**: [EVA-Planner: Environmental Adaptive Quadrotor Planning, Lun Quan, Zhiwei Zhang, Chao Xu and Fei Gao](https://ieeexplore.ieee.org/document/9561759) accepted by 2021 IEEE International Conference on Robotics and Automation (ICRA).

**Video Links**: [Google](https://www.youtube.com/watch?v=HcwBNcah0eo&t=4s), [Bilibili](https://www.bilibili.com/video/BV1Zz4y1C7rt)(for Mainland China)

## 1. File Structrue
- All planning algorithms along with other key modules, such as mapping, are implemented in **adaptive_planner**
    - **path_searching**: includes multi-layer planner (A*, low-MPC and high-MPCC).

    - **path_env**: includes online mapping algorithms for the planning system (grid map and **ESDF**(Euclidean signed distance filed)). 
    
    - **path_manage**: High-level modules that schedule and call the mapping and planning algorithms. Interfaces for launching the whole system, as well as the configuration files are contained here

## 2. Compilation
**Requirements**: ubuntu 16.04, 18.04 or 20.04 with ros-desktop-full installation

**Step 1**. Install [Armadillo](http://arma.sourceforge.net/), which is required by **uav_simulator**.
```
sudo apt-get install libarmadillo-dev
```

**Step 2**. We use [NLopt](https://nlopt.readthedocs.io/en/latest/NLopt_Installation/) to solve the non-linear optimization problem. Please follow the Installation process in NLopt Documentation.

**Step 3**. Clone the code from github.
```
git clone https://github.com/ZJU-FAST-Lab/EVA-planner.git
```
**Step 4**. Compile.
```
cd EVA-planner
catkin_make
```

## 3. Use GPU or not
Packages in this repo, local_sensing have GPU, CPU two different versions. By default, they are in CPU version for better compatibility. By changing
 ```
 set(ENABLE_CUDA false)
 ```
in the _CMakeList.txt_ in **local_sensing** packages, to
 ```
 set(ENABLE_CUDA true)
 ```

CUDA will be turned-on to generate depth images as a real depth camera does. 

Please remember to also change the 'arch' and 'code' flags in the line of 
```
    set(CUDA_NVCC_FLAGS 
      -gencode arch=compute_61,code=sm_61;
    ) 
``` 
in _CMakeList.txt_, if you encounter compiling error due to different Nvidia graphics card you use. You can check the right code [here](https://github.com/tpruvot/ccminer/wiki/Compatibility).
 
Don't forget to re-compile the code!

**local_sensing** is the simulated sensors. If ```ENABLE_CUDA``` **true**, it mimics the depth measured by stereo cameras and renders a depth image by GPU. If ```ENABLE_CUDA``` **false**, it will publish pointclouds with no ray-casting. Our local mapping module automatically selects whether depth images or pointclouds as its input.

For installation of CUDA, please go to [CUDA ToolKit](https://developer.nvidia.com/cuda-toolkit)

## 4. Run a simple example.
```
source devel/setup.bash
roslaunch plan_manage simulation.launch
```
Then you can enter **G** with the keyboard and use the mouse to select a target.

# Acknowledgements
- The framework of this repository is based on [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) by Zhou Boyu who achieves impressive proformance on quaorotor local planning.
- We use [NLopt](https://nlopt.readthedocs.io/en/latest/) for non-linear optimization.
- The hardware architecture is based on an open source implemation from [Teach-Repeat-Replan](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan).
- The benchmark compared in our paper is [ICRA2020_RG_SDDM](https://github.com/zhl355/ICRA2020_RG_SDDM/tree/code). 

# Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

# Maintaince
For any technical issues, please contact Lun Quan (lunquan@zju.edu.cn) or Fei GAO (fgaoaa@zju.edu.cn).

For commercial inquiries, please contact Fei GAO (fgaoaa@zju.edu.cn).
