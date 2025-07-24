# AGV_Pro
ROS2 packages for AGV_Pro

> Software environment for Jetson Orin Nano

```
ubuntu 22.04
ros2 humble
gazebo 11
```

# Installation

Create workspace and clone the repository.

```
git clone https://github.com/jiaweilong66/agv_pro.git
```

Install dependencies

```
cd ~/agv_pro

rosdep install --from-paths src --ignore-src -r -y
```

Build workspace

```
cd ~/agv_pro

colcon build
```

Setup the workspace

```
source ~/agv_pro/install/setup.bash
```

```
apt install ros-$ROS_DISTRO-gazebo-ros-pkgs

sudo apt install ros-$ROS_DISTRO-ros2-controllers

sudo apt install ros-humble-teleop-twist-keyboard
```

# Update to new version

```
cd ~/myagv_ros2/src

git pull

cd ..

colcon build
```

# Start

```
ros2 launch agv_pro_gazebo agv_pro_gazebo.launch.py
```
