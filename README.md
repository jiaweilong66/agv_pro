<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="agv_pro">
  <!-- Namespace 设定 -->
  <xacro:arg name="namespace" default=""/>
  <xacro:property name="namespace" value="$(arg namespace)"/>

  <!-- 基座 链接与关节 -->
  <link name="${namespace}base_footprint"/>

  <joint name="${namespace}base_joint" type="fixed">
    <parent link="${namespace}base_footprint"/>
    <child  link="${namespace}base_link"/>
    <origin xyz="0 0 0.020" rpy="0 0 0"/>
  </joint>

  <link name="${namespace}base_link">
    <inertial>
      <origin xyz="-0.0076254 -0.00023134 0.06693" rpy="0 0 0"/>
      <mass    value="19.236"/>
      <inertia
        ixx="0.14436" ixy="0.0012037" ixz="0.0019458"
        iyy="0.24191" iyz="0.0044629" izz="0.33755"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://agv_pro_description/meshes/base_link.stl"/>
      </geometry>
      <material>
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://agv_pro_description/meshes/base_link.stl"/>
      </geometry>
    </collision>
  </link>

  <!-- 右后轮 -->
  <link name="${namespace}right_rear_wheel_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass    value="0.21659"/>
      <inertia
        ixx="0.00051181" ixy="2.1577e-08" ixz="2.538e-07"
        iyy="0.00097519" iyz="-2.3635e-07" izz="0.00051178"/>
    </inertial>
    <visual>
      <origin  xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://agv_pro_description/meshes/wheel_rb_link.stl"/>
      </geometry>
      <material>
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin  xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://agv_pro_description/meshes/wheel_rb_link.stl"/>
      </geometry>
    </collision>
  </link>
  <joint name="${namespace}right_rear_wheel_joint" type="continuous">
    <parent link="${namespace}base_link"/>
    <child  link="${namespace}right_rear_wheel_link"/>
    <origin xyz="-0.171806101587598 -0.179900399999999 0.0518836514526621" rpy="0 0 0"/>
    <axis   xyz="0 1 0"/>
  </joint>

  <!-- 右前轮 -->
  <link name="${namespace}right_front_wheel_link">
    <inertial>
      <origin xyz="0.000066563 -0.019725 0.000083836" rpy="0 0 0"/>
      <mass    value="0.21659122149244"/>
      <inertia
        ixx="0.00051181" ixy="2.1577e-08" ixz="2.538e-07"
        iyy="0.00097519" iyz="-2.3635e-07" izz="0.00051178"/>
    </inertial>
    <visual>
      <origin  xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://agv_pro_description/meshes/wheel_rf_link.stl"/>
      </geometry>
      <material>
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin  xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://agv_pro_description/meshes/wheel_rf_link.stl"/>
      </geometry>
    </collision>
  </link>
  <joint name="${namespace}right_front_wheel_joint" type="continuous">
    <parent link="${namespace}base_link"/>
    <child  link="${namespace}right_front_wheel_link"/>
    <origin xyz="-0.17181 0.1799 0.051884" rpy="0 0 0"/>
    <axis   xyz="0 1 0"/>
  </joint>

  <!-- 左前轮 -->
  <link name="${namespace}left_front_wheel_link">
    <inertial>
      <origin xyz="0.0000014671 -0.019803 0.0000043218" rpy="0 0 0"/>
      <mass    value="0.3015"/>
      <inertia
        ixx="0.00052475" ixy="-2.2533e-07" ixz="-4.1904e-07"
        iyy="0.00099948" iyz="7.5332e-08" izz="0.00052362"/>
    </inertial>
    <visual>
      <origin  xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://agv_pro_description/meshes/wheel_lf_link.stl"/>
      </geometry>
      <material>
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin  xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://agv_pro_description/meshes/wheel_lf_link.stl"/>
      </geometry>
    </collision>
  </link>
  <joint name="${namespace}left_front_wheel_joint" type="continuous">
    <parent link="${namespace}base_link"/>
    <child  link="${namespace}left_front_wheel_link"/>
    <origin xyz="0.17128 0.1799 0.052" rpy="0 0 0"/>
    <axis   xyz="0 1 0"/>
  </joint>

  <!-- 左后轮 -->
  <link name="${namespace}left_rear_wheel_link">
    <inertial>
      <origin xyz="-0.0000024454 0.019725 -0.0000043121" rpy="0 0 0"/>
      <mass    value="0.29613"/>
      <inertia
        ixx="0.00051227" ixy="-2.0628e-07" ixz="-4.9074e-07"
        iyy="0.0009752" iyz="1.173e-07" izz="0.00051131"/>
    </inertial>
    <visual>
      <origin  xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://agv_pro_description/meshes/wheel_lb_link.stl"/>
      </geometry>
      <material>
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin  xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://agv_pro_description/meshes/wheel_lb_link.stl"/>
      </geometry>
    </collision>
  </link>
  <joint name="${namespace}left_rear_wheel_joint" type="continuous">
    <parent link="${namespace}base_link"/>
    <child  link="${namespace}left_rear_wheel_link"/>
    <origin xyz="0.17128 -0.1799 0.052" rpy="0 0 0"/>
    <axis   xyz="0 1 0"/>
  </joint>

  <!-- 激光雷达及摄像头 -->
  <link name="${namespace}laser_link">
    <inertial>
      <origin xyz="-0.0035142 -0.000028248 0.0010013" rpy="0 0 0"/>
      <mass    value="0.049095"/>
      <inertia
        ixx="0.000020572" ixy="8.5013e-08" ixz="2.0871e-07"
        iyy="0.000020483" iyz="-4.2154e-09" izz="0.000034612"/>
    </inertial>
    <visual>
      <origin  xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://agv_pro_description/meshes/laser_link.stl"/>
      </geometry>
      <material>
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin  xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://agv_pro_description/meshes/laser_link.stl"/>
      </geometry>
    </collision>
  </link>
  <joint name="${namespace}lidar_joint" type="fixed">
    <parent link="${namespace}base_link"/>
    <child  link="${namespace}laser_link"/>
    <origin xyz="0.17891 0 0.20928" rpy="0 0 0"/>
  </joint>

  <link name="${namespace}camera_link"/>
  <joint name="${namespace}camera_joint" type="fixed">
    <parent link="${namespace}base_link"/>
    <child  link="${namespace}camera_link"/>
    <origin xyz="0.23191 0 0.14928" rpy="0 0 0"/>
  </joint>

  <!-- 激光雷达安装架，加高 0.10m -->
  <link  name="laser_mount"/>
  <joint name="laser_mount_joint" type="fixed">
    <parent link="laser_link"/>
    <child  link="laser_mount"/>
    <origin xyz="0 0 0.10" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo 插件 -->
  <xacro:include filename="$(find agv_pro_gazebo)/urdf/parts/gazebo_control_plugin.xacro"/>
  <xacro:include filename="$(find agv_pro_gazebo)/urdf/parts/gazebo_sensor_plugin.xacro"/>
  <xacro:gazebo_sensor_plugin/>
  <xacro:gazebo_control_plugin/>
</robot>
