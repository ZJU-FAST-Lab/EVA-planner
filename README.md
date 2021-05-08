# EVA-planner

**EVA**-planner: an **E**n**V**ironmental **A**daptive Gradient-based Local Planner for Quadrotors.

## 0. Overview
**Author**: Lun Quan, Zhiwei Zhang, Xingguang Zhong, Chao Xu and [Fei Gao](https://ustfei.com/) from [ZJU FAST Lab](http://www.kivact.com/).

**Related Paper**: [EVA-Planner: Environmental Adaptive Quadrotor Planning, Lun Quan, Zhiwei Zhang, Chao Xu and Fei Gao](http://arxiv.org/abs/2011.04246) accepted by ICRA 2021.

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

**Step 2**. Clone the code from github.
```
git clone https://github.com/ZJU-FAST-Lab/EVA-planner.git
```
**Step 3**. Compile.
```
cd EVA-planner
catkin_make
```

## 3. Run a simple example.
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
