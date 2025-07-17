<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">

  <xacro:macro name="wheel" params="name mesh origin_xyz origin_rpy mass ixx ixy ixz iyy iyz izz">

    <link name="${name}_link">
      <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="${mass}"/>
        <inertia
          ixx="${ixx}"
          ixy="${ixy}"
          ixz="${ixz}"
          iyy="${iyy}"
          iyz="${iyz}"
          izz="${izz}"/>
      </inertial>
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <mesh filename="${mesh}"/>
        </geometry>
        <material name="gray">
          <color rgba="0.3 0.3 0.3 1"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <mesh filename="${mesh}"/>
        </geometry>
      </collision>
    </link>

    <joint name="${name}_joint" type="continuous">
      <origin xyz="${origin_xyz}" rpy="${origin_rpy}"/>
      <parent link="base_link"/>
      <child link="${name}_link"/>
      <axis xyz="0 1 0"/>
      <!-- 增大阻尼并加摩擦，抑制抖动 -->
      <dynamics damping="1.0" friction="0.5"/>
    </joint>

    <!-- Correct transmission for ROS2 -->
    <transmission name="${name}_trans">
      <type>transmission_interface/SimpleTransmission</type>
      <joint name="${name}_joint">
        <hardwareInterface>hardware_interface/velocity</hardwareInterface>
      </joint>
      <actuator name="${name}_motor">
        <mechanicalReduction>1</mechanicalReduction>
        <hardwareInterface>hardware_interface/velocity</hardwareInterface>
      </actuator>
    </transmission>

    <gazebo reference="${name}_link">
      <!-- 增大摩擦，软化接触刚度，提高阻尼 -->
      <mu1>1.0</mu1>
      <mu2>1.0</mu2>
      <kp>50000.0</kp>
      <kd>20.0</kd>
      <material>Gazebo/Black</material>
    </gazebo>

  </xacro:macro>
</robot>
