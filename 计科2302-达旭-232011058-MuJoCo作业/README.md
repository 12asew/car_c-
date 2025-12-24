# MuJoCo MPC  汽⻋仪表盘项⽬
## 项⽬信息
- **学号**:232011058
- **姓名**:达旭
- **班级**：计科2302
- **完成日期**：2025年12月24日

## 项目概述
本系统旨在通过MuJoCo物理引擎模拟真实车辆的运动特性，并实时提取仿真数据，通过OpenGL渲染技术将数据以汽车仪表盘的形式可视化展示。项目不仅实现了基础的物理仿真功能，还创新地将传统仪表盘UI集成到3D仿真环境中

## 核心功能
物理仿真模块
车辆动力学模拟：基于MuJoCo引擎实现车辆运动、碰撞检测等物理特性
自定义场景设计：使用MJCF格式创建包含车辆、地面、障碍物的仿真环境
实时状态更新：每帧更新车辆位置、速度、加速度等物理参数
数据监控模块
多维度数据采集：实时获取速度、转速、位置、油量、温度等关键指标
数据转换处理：实现单位转换（m/s→km/h）、数据平滑等预处理
状态预警机制：转速过高、温度异常等情况的自动检测
可视化界面模块
传统仪表盘组件：
速度表（0-200 km/h带刻度指针）
转速表（0-8000 RPM含红线预警区）
数字信息面板（油量、温度进度条）
实时数据绑定：UI组件与仿真数据动态关联
视觉美化效果：半透明背景、平滑动画、色彩预警提示
技术实现特色
OpenGL 2D覆盖渲染：在3D物理场景上叠加2D UI界面
模块化架构设计：数据提取、渲染逻辑、场景管理分离

物理仿真层 (MuJoCo)
    ↓
数据提取层 (C++数据接口)
    ↓
业务逻辑层 (数据处理转换)
    ↓
渲染表现层 (OpenGL UI渲染)
    ↓
用户界面层 (汽车仪表盘)

## 项目亮点
## 技术创新
物理引擎与UI的深度集成：将工业级物理仿真与用户界面完美结合
实时数据流水线：从物理计算到视觉呈现的毫秒级延迟
模块化可扩展架构：支持功能组件的灵活添加和替换

## 环境要求
- 操作系统：Ubuntu 22.04
- 编译器： gcc 11.3.0
- CMake： 3.22.1

<h1>
  <a href="#"><img alt="MuJoCo MPC" src="docs/assets/banner.png" width="100%"></a>
</h1>

<p>
  <a href="https://github.com/google-deepmind/mujoco_mpc/actions/workflows/build.yml?query=branch%3Amain" alt="GitHub Actions">
    <img src="https://img.shields.io/github/actions/workflow/status/google-deepmind/mujoco_mpc/build.yml?branch=main">
  </a>
  <a href="https://github.com/google-deepmind/mujoco_mpc/blob/main/LICENSE" alt="License">
    <img src="https://img.shields.io/github/license/google-deepmind/mujoco_mpc">
  </a>
</p>

**MuJoCo MPC (MJPC)** is an interactive application and software framework for
real-time predictive control with [MuJoCo](https://mujoco.org/), developed by
Google DeepMind.

MJPC allows the user to easily author and solve complex robotics tasks, and
currently supports multiple shooting-based planners. Derivative-based methods include iLQG and
Gradient Descent, while derivative-free methods include a simple yet very competitive planner
called Predictive Sampling.

- [Overview](#overview)
- [Graphical User Interface](#graphical-user-interface)
- [Installation](#installation)
  - [macOS](#macos)
  - [Ubuntu](#ubuntu)
  - [Build Issues](#build-issues)
- [Predictive Control](#predictive-control)
- [Contributing](#contributing)
- [Known Issues](#known-issues)
- [Citation](#citation)
- [Acknowledgments](#acknowledgments)
- [License and Disclaimer](#license-and-disclaimer)

## Overview

To read the paper describing this software package, please see our
[preprint](https://arxiv.org/abs/2212.00541).

For a quick video overview of MJPC, click below.

[![Video](http://img.youtube.com/vi/Bdx7DuAMB6o/hqdefault.jpg)](https://dpmd.ai/mjpc)

For a longer talk at the MIT Robotics Seminar in December 2022 describing our results, click
below.

[![2022Talk](http://img.youtube.com/vi/2xVN-qY78P4/hqdefault.jpg)](https://www.youtube.com/watch?v=2xVN-qY78P4)

A more recent, December 2023 talk at the IEEE Technical Committee on Model-Based Optimization
is available here:

[![2023Talk](https://img.youtube.com/vi/J-JO-lgaKtw/hqdefault.jpg)](https://www.youtube.com/watch?v=J-JO-lgaKtw&t=0s)

### Example tasks

Quadruped task:

[![Quadruped](http://img.youtube.com/vi/esLuwaWz4oE/hqdefault.jpg)](https://www.youtube.com/watch?v=esLuwaWz4oE)


Bimanual manipulation:

[![Bimanual](http://img.youtube.com/vi/aCNCKVThKIE/hqdefault.jpg)](https://www.youtube.com/watch?v=aCNCKVThKIE)


Rubik's cube 10-move unscramble:

[![Unscramble](http://img.youtube.com/vi/ZRRvVWV-Muk/hqdefault.jpg)](https://www.youtube.com/watch?v=ZRRvVWV-Muk)

Humanoid motion-capture tracking:

[![Tracking](http://img.youtube.com/vi/tEBVK-MO1Sw/hqdefault.jpg)](https://www.youtube.com/watch?v=tEBVK-MO1Sw)

## Graphical User Interface

For a detailed dive of the graphical user interface, see the
[MJPC GUI](docs/GUI.md) documentation.

## Installation
MJPC is tested with [Ubuntu 20.04](https://releases.ubuntu.com/focal/) and [macOS-12](https://www.apple.com/by/macos/monterey/). In principle, other versions and Windows operating system should work with MJPC, but these are not tested.

### Prerequisites
Operating system specific dependencies:

#### macOS
Install [Xcode](https://developer.apple.com/xcode/).

Install `ninja` and `zlib`:
```sh
brew install ninja zlib
```

#### Ubuntu 20.04
```sh
sudo apt-get update && sudo apt-get install cmake libgl1-mesa-dev libxinerama-dev libxcursor-dev libxrandr-dev libxi-dev ninja-build zlib1g-dev clang-12
```

### Clone MuJoCo MPC
```sh
git clone https://github.com/google-deepmind/mujoco_mpc
```

### Build and Run MJPC GUI application
1. Change directory:
```sh
cd mujoco_mpc
```

2. Create and change to build directory:
```sh
mkdir build
cd build
```

3. Configure:

#### macOS-12
```sh
cmake .. -DCMAKE_BUILD_TYPE:STRING=Release -G Ninja -DMJPC_BUILD_GRPC_SERVICE:BOOL=ON
```

#### Ubuntu 20.04
```sh
cmake .. -DCMAKE_BUILD_TYPE:STRING=Release -G Ninja -DCMAKE_C_COMPILER:STRING=clang-12 -DCMAKE_CXX_COMPILER:STRING=clang++-12 -DMJPC_BUILD_GRPC_SERVICE:BOOL=ON
```
**Note: gRPC is a large dependency and can take 10-20 minutes to initially download.**

4. Build
```sh
cmake --build . --config=Release
```

6. Run GUI application
```sh
cd bin
./mjpc
```

### Build and Run MJPC GUI application using VSCode
We recommend using [VSCode](https://code.visualstudio.com/) and 2 of its
extensions ([CMake Tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools)
and [C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools))
to simplify the build process.

1. Open the cloned directory `mujoco_mpc`.
2. Configure the project with CMake (a pop-up should appear in VSCode)
3. Set compiler to `clang-12`.
4. Build and run the `mjpc` target in "release" mode (VSCode defaults to
   "debug"). This will open and run the graphical user interface.

### Build Issues
If you encounter build issues, please see the
[Github Actions configuration](https://github.com/google-deepmind/mujoco_mpc/blob/main/.github/workflows/build.yml).
This provides the exact setup we use for building MJPC for testing with Ubuntu 20.04 and macOS-12.

# Python API
We provide a simple Python API for MJPC. This API is still experimental and expects some more experience from its users. For example, the correct usage requires that the model (defined in Python) and the MJPC task (i.e., the residual and transition functions defined in C++) are compatible with each other. Currently, the Python API does not provide any particular error handling for verifying this compatibility and may be difficult to debug without more in-depth knowledge about MuJoCo and MJPC.

## Installation

### Prerequisites
1. Build MJPC (see instructions above).

2. Python 3.10

3. (Optionally) Create a conda environment with **Python 3.10**:
```sh
conda create -n mjpc python=3.10
conda activate mjpc
```

4. Install MuJoCo
```sh
pip install mujoco
```

### Install API
Next, change to the python directory:
```sh
cd python
```

Install the Python module:
```sh
python setup.py install
```

Test that installation was successful:
```sh
python "mujoco_mpc/agent_test.py"
```

Example scripts are found in `python/mujoco_mpc/demos`. For example from `python/`:
```sh
python mujoco_mpc/demos/agent/cartpole_gui.py
```
will run the MJPC GUI application using MuJoCo's passive viewer via Python.

### Python API Installation Issues
If your installation fails or is terminated prematurely, we recommend deleting the MJPC build directory and starting from scratch as the build will likely be corrupted. Additionally, delete the files generated during the installation process from the `python/` directory.

## Predictive Control

See the [Predictive Control](docs/OVERVIEW.md) documentation for more
information.

## Contributing

See the [Contributing](docs/CONTRIBUTING.md) documentation for more information.

## Known Issues

MJPC is not production-quality software, it is a **research prototype**. There
are likely to be missing features and outright bugs. If you find any, please
report them in the [issue tracker](https://github.com/google-deepmind/mujoco_mpc/issues).
Below we list some known issues, including items that we are actively working
on.

- We have not tested MJPC on Windows, but there should be no issues in
  principle.
- Task specification, in particular the setting of norms and their parameters in
  XML, is a bit clunky. We are still iterating on the design.
- The Gradient Descent search step is proportional to the scale of the cost
  function and requires per-task tuning in order to work well. This is not a bug
  but a property of vanilla gradient descent. It might be possible to ameliorate
  this with some sort of gradient normalisation, but we have not investigated
  this thoroughly.

## Citation

If you use MJPC in your work, please cite our accompanying [preprint](https://arxiv.org/abs/2212.00541):

```bibtex
@article{howell2022,
  title={{Predictive Sampling: Real-time Behaviour Synthesis with MuJoCo}},
  author={Howell, Taylor and Gileadi, Nimrod and Tunyasuvunakool, Saran and Zakka, Kevin and Erez, Tom and Tassa, Yuval},
  archivePrefix={arXiv},
  eprint={2212.00541},
  primaryClass={cs.RO},
  url={https://arxiv.org/abs/2212.00541},
  doi={10.48550/arXiv.2212.00541},
  year={2022},
  month={dec}
}
```

## Acknowledgments

The main effort required to make this repository publicly available was
undertaken by [Taylor Howell](https://thowell.github.io/) and the Google
DeepMind Robotics Simulation team.

## License and Disclaimer

All other content is Copyright 2022 DeepMind Technologies Limited and licensed
under the Apache License, Version 2.0. A copy of this license is provided in the
top-level LICENSE file in this repository. You can also obtain it from
https://www.apache.org/licenses/LICENSE-2.0.

This is not an officially supported Google product.
