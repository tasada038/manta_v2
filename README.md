# manta_v2
ROS 2 package suite of manta_v2.

## Requirements
- Manta V2
- Linux OS
    - Ubuntu 20.04 Laptop/Desktop PC
- ROS
    - Foxy Fitzroy

## Install & Build
The following commands download a package from a remote repository and install it in your colcon workspace.

```
mkdir -p ~/manta_ws/src
cd ~/manta_ws/src
git clone https://github.com/tasada038/manta_v2.git
cd ~/manta_ws && colcon build --symlink-install
```

## Quick Start
If you have not installed the 'ros-foxy-joy-linux' package, run the following command to install the ros-foxy-joy-linux package.
```
sudo apt install ros-foxy-joy-linux
sudo apt install ros-foxy-joy-linux-dbgsym
```

In the first shell, run the joy_linux_node.
```
ros2 run joy_linux joy_linux_node 
```

In the second shell, run the bringup launch file.
```
ros2 launch manta_v2_bringup manta_bringup.launch.py
```

In the third shell, run the controller launch file.
```
ros2 launch manta_v2_controller joint_states.launch.py 
```

Next, connect the robot and PC with a tether cable and connect to Jetson Nano on the robot side by SSH.

```
ssh jetson@192.168.13.26
```

(* default IP of Jetson Nano is 192.168.13.26)

And, run the control launch file.
```
ros2 launch manta_v2_controller robot_control.launch.py
```

In addition, in the second shell on the robot side(Jetson Nano), run the servo node.
```
ros2 run manta_v2_controller servo_node
```

## Packages
- manta_v2_description

This package contains model files (urdf, stl) for loading manta_v2 in ROS2 and Rviz2 environments.

- manta_v2_bringup

This package contains launch files for loading manta_v2 in ROS2 and Rviz2 environments.

- manta_v2_controller

This package controls manta_v2 swimming and walking.

- manta_v2_gazebo_ros2_control

This package simulates manta_v2 with Gazebo

- manta_v2_moveit_config

This package contains manta_v2 configuration files (yaml, srdf) to use MoveIt2 package in ROS2 environment.

- manta_v2_run_move_group

This package for moving manta_v2 using Move Group C++ Interface.

## License
This repository is licensed under the Apache 2.0, see LICENSE for details.