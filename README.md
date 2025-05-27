# AGV_Pro
ROS2 packages for AGV_Pro

> Software environment for Jetson Orin Nano

```
ubuntu 22.04
ros2 humble
```

# Installation

Create workspace and clone the repository.

```
git clone https://github.com/elephantrobotics/agv_pro_ros2.git agv_pro_ros2/src
```

Install dependencies

```
cd ~/agv_pro_ros2

rosdep install --from-paths src --ignore-src -r -y
```

Build workspace

```
cd ~/agv_pro_ros2

colcon build
```

Setup the workspace

```
source ~/agv_pro_ros2/install/local_setup.bash
```

# Update to new version

```
cd ~/myagv_ros2/src

git pull

cd ..

colcon build
```

