<gazebo>
    <plugin name="rosa_controller" filename="libgazebo_ros_omni_drive.so">
      <commandTopic>cmd_vel</commandTopic>
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <odometryRate>20.0</odometryRate>
      <robotBaseFrame>base_footprint</robotBaseFrame>
      <wheel_radius>0.075</wheel_radius>
      <base_length>0.65</base_length>
      <base_width>0.65</base_width>
      <front_left_joint>wheel_front_left_joint</front_left_joint>
      <front_right_joint>wheel_front_right_joint</front_right_joint>
      <rear_left_joint>wheel_back_left_joint</rear_left_joint>
      <rear_right_joint>wheel_back_right_joint</rear_right_joint>
      <wheel_max_speed> 20.0 </wheel_max_speed>
      <wheel_acceleration> 10.0</wheel_acceleration>
      <joint_config>1 1 -1 -1</joint_config>
    </plugin>
  </gazebo>
