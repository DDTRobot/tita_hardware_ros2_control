<p align="center"><strong>tita_hardware</strong></p>
<p align="center"><a href="https://github.com/${YOUR_GIT_REPOSITORY}/blob/main/LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache%202.0-orange"/></a>
<img alt="language" src="https://img.shields.io/badge/language-c++-red"/>
<img alt="platform" src="https://img.shields.io/badge/platform-linux-l"/>
</p>
<p align="center">
    语言：<a href="./docs/docs_en/README_EN.md"><strong>English</strong></a> / <strong>中文</strong>
</p>

## Description

Tita hardware ros2 control.

## Prerequisites

- **Operating System**: Ubuntu 22.04
- **ROS 2**: Humble
  
## Dependencies

```bash
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
```
## Build Package

```bash
ssh robot@192.168.42.1
mkdir tita_ws/src && cd tita_ws/src
git clone https://github.com/DDTRobot/TITA_Description.git

git clone https://github.com/DDTRobot/TITA_ROS2_Control_Sim.git
colcon build --packages-up-to hw_bringup 
source install/setup.bash
ros2 launch hw_bringup hw_bringup.launch.py sim_env:=gazebo 
```
In the launch file, the default controller to start is `effort_controllers/JointGroupEffortController`. Create a new terminal and enter the following command to confirm if the controller is working properly.
```bash
ros2 topic pub -1 /tita_hw/effort_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5]}"
```

If you want to write your own controller, you can modify it according to `template_ros2_controller`.