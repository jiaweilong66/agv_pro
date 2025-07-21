<?xml version="1.0" ?>
<world name="default">
  <!-- 引用 Gazebo 默认插件 -->
  <include file="$(find gazebo_ros)/worlds/empty.world"/>

  <physics type="ode">
    <max_step_size>0.01</max_step_size>  <!-- 物理引擎步长 -->
    <real_time_update_rate>100</real_time_update_rate>  <!-- 仿真实时更新率 -->
  </physics>

  <!-- 创建外部长方体，宽810cm，长340cm，并在底部添加缺口 -->
  <model name="outer_rectangle">
    <link name="outer_link">
      <collision name="collision">
        <geometry>
          <box>
            <size>8.10 3.40 0.2</size>  <!-- 宽810cm，长340cm，高20cm -->
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.9</mu>
              <mu2>0.9</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>8.10 3.40 0.2</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Gray</name>
          </script>
        </material>
      </visual>
    </link>

    <!-- 在底部中间添加缺口 -->
    <pose>0 0 -0.1 0 0 0</pose> <!-- 设置位置 -->
    <plugin name="door_plugin" filename="libgazebo_ros_planar_move.so">
      <!-- 缺口参数 -->
      <link>outer_link</link>
      <pose>0 1.65 -0.1 0 0 0</pose>  <!-- 缺口位置 -->
      <size>1.5 0.2 0.2</size>  <!-- 缺口大小150cm x 20cm -->
    </plugin>
  </model>

  <!-- 创建内部桌子模型，长120cm，宽450cm -->
  <model name="inner_table">
    <link name="table_link">
      <collision name="collision">
        <geometry>
          <box>
            <size>1.20 4.50 0.2</size>  <!-- 长120cm，宽450cm，高20cm -->
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.9</mu>
              <mu2>0.9</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>1.20 4.50 0.2</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Gray</name>
          </script>
        </material>
      </visual>
    </link>
    <pose>0 0 0.1 0 0 0</pose>  <!-- 将桌子放置到正确的位置 -->
  </model>

</world>
