#include "agv_pro_base/agv_pro_driver.h"

std::array<double, 36> odom_pose_covariance = {
  {1e-9, 0, 0, 0, 0, 0,
  0, 1e-3, 1e-9, 0, 0, 0,
  0, 0, 1e6, 0, 0, 0,
  0, 0, 0, 1e6, 0, 0,
  0, 0, 0, 0, 1e6, 0,
  0, 0, 0, 0, 0, 1e-9} };

std::array<double, 36> odom_twist_covariance = {
  {1e-9, 0, 0, 0, 0, 0,
  0, 1e-3, 1e-9, 0, 0, 0,
  0, 0, 1e6, 0, 0, 0,
  0, 0, 0, 1e6, 0, 0,
  0, 0, 0, 0, 1e6, 0,
  0, 0, 0, 0, 0, 1e-9} };

uint16_t crc16_ibm(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]);
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x0001)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc = crc >> 1;
    }
  }
  return crc;
}

void AGV_PRO::set_auto_report(){

  std::array<uint8_t, 14> buf = {
    0xFE, 0xFE, 0x0b, 0x23,
    0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
  };

  uint16_t crc = crc16_ibm(buf.data(), 12);
  buf[12] = (crc >> 8) & 0xff;
  buf[13] = crc & 0xff;

  std::vector<uint8_t> data_vec(buf.begin(), buf.end());

  auto port = serial_driver_->port();

  try
  {
    size_t bytes_transmit_size = port->send(data_vec);
    std::stringstream ss;
    for (auto b : data_vec) {
      ss << std::hex << std::uppercase << std::setfill('0') << std::setw(2)
         << static_cast<int>(b) << " ";
    }
    RCLCPP_INFO(this->get_logger(), "Sent %ld bytes: [%s]", bytes_transmit_size, ss.str().c_str());
  }
  catch(const std::exception &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Error Transmiting from serial port:%s",ex.what());
  }

}

void AGV_PRO::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  linearX = std::clamp(msg->linear.x, -1.5, 1.5);
  linearY = std::clamp(msg->linear.y, -1.0, 1.0);
  angularZ = std::clamp(msg->angular.z, -1.0, 1.0);

  int16_t x_send = static_cast<int16_t>(linearX * 100);
  int16_t y_send = static_cast<int16_t>(linearY * 100);
  int16_t rot_send = static_cast<int16_t>(angularZ * 100);

  uint8_t buf[14] = { 0xfe,0xfe,0x0b,0x21 };

  buf[4] = (x_send >> 8) & 0xff;
  buf[5] = x_send & 0xff;
  buf[6] = (y_send >> 8) & 0xff;
  buf[7] = y_send & 0xff;
  buf[8] = (rot_send >> 8) & 0xff;
  buf[9] = rot_send & 0xff;
  buf[10] = 0x00;
  buf[11] = 0x00;

  uint16_t crc = crc16_ibm(buf, 12);
  buf[12] = (crc >> 8) & 0xff;
  buf[13] = crc & 0xff;

  std::vector<uint8_t> data_vec(buf, buf + sizeof(buf));

  auto port = serial_driver_->port();

  try
  {
    port->send(data_vec);

    //debug************************************
    // size_t bytes_transmit_size = port->send(data_vec);
    // std::stringstream ss;
    // for (auto b : data_vec) {
    //   ss << std::hex << std::uppercase << std::setfill('0') << std::setw(2)
    //      << static_cast<int>(b) << " ";
    // }
    // RCLCPP_INFO(this->get_logger(), "Sent %ld bytes: [%s]", bytes_transmit_size, ss.str().c_str());
    //debug************************************

  }
  catch(const std::exception &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Error Transmiting from serial port:%s",ex.what());
  }
}

bool AGV_PRO::readData()
{
  std::vector<uint8_t> buf_header(1);
  std::vector<uint8_t> buf_length(1);
  std::vector<uint8_t> data_buf(RECEIVE_DATA_SIZE-3);

  auto port = serial_driver_->port();
  
  while (true)
  {
    size_t ret = port->receive(buf_header);
    if (ret != 1 || buf_header[0] != 0xfe) {
      continue;
    }

    ret = port->receive(buf_header);
    if (ret == 1 && buf_header[0] == 0xfe) {
      break; 
    }
  }

  size_t ret = port->receive(buf_length);

  if (buf_length[0] != 0x0b) {
    RCLCPP_ERROR(this->get_logger(), "The received length is incorrect:%u", buf_length[0]);
    return false;
  }

  ret = port->receive(data_buf);
  if (ret != data_buf.size())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to receive full payload");
    return false;
  }

  std::vector<uint8_t> recv_buf;
  recv_buf.push_back(0xFE);
  recv_buf.push_back(0xFE);
  recv_buf.push_back(0x0B);
  recv_buf.insert(recv_buf.end(), data_buf.begin(), data_buf.end());
  
  //debug************************************
  // std::stringstream ss;
  // for (const auto& byte : recv_buf) {
  //     ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
  //       << static_cast<int>(byte) << " ";
  // }
  // RCLCPP_INFO(this->get_logger(), "recv_buf: [%s]", ss.str().c_str());
  //debug************************************

  if (recv_buf[3] != 0x25) {
    // RCLCPP_WARN(this->get_logger(), "Command error:0x%02X", recv_buf[2]);
    return false;
  }

  uint16_t received_crc = recv_buf[13] | (recv_buf[12] << 8);
  uint16_t computed_crc = crc16_ibm(recv_buf.data(), 12);

  if (received_crc != computed_crc) {
    RCLCPP_WARN(this->get_logger(), "CRC error: received 0x%04X, calculated 0x%04X", received_crc, computed_crc);
    return false;
  }

  vx = static_cast<double>(static_cast<int8_t>(recv_buf[4])) * 0.01;
  vy = static_cast<double>(static_cast<int8_t>(recv_buf[5])) * 0.01;
  vtheta = static_cast<double>(static_cast<int8_t>(recv_buf[6])) * 0.01;

  motor_status = recv_buf[7];
  motor_error  = recv_buf[8];
  battery_voltage = static_cast<float>(recv_buf[9]) / 10.0f;
  enable_status = recv_buf[10];

  return true;
}

void AGV_PRO::publisherVoltage()
{
  std_msgs::msg::Float32 voltage_msg,voltage_backup_msg;
  voltage_msg.data = battery_voltage;
  pub_voltage->publish(voltage_msg);
}

void AGV_PRO::publisherOdom(double dt)
{   
  geometry_msgs::msg::TransformStamped odom_trans;
  odom_trans.header.stamp = this->get_clock()->now();
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_footprint";

  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(quat);

  double delta_x = (vx * cos(theta) - vy * sin(theta)) * dt;
  double delta_y = (vx * sin(theta) + vy * cos(theta)) * dt;
  double delta_th = vtheta * dt;

  x += delta_x;
  y += delta_y;
  theta += delta_th;

  odom_trans.transform.translation.x = x; 
  odom_trans.transform.translation.y = y; 
  odom_trans.transform.translation.z = 0.0;

  odom_trans.transform.rotation = odom_quat;

  odomBroadcaster->sendTransform(odom_trans);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = this->get_clock()->now();;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  odom.pose.covariance = odom_pose_covariance;

  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vtheta;
  odom.twist.covariance = odom_twist_covariance;

  pub_odom->publish(odom);
}

void AGV_PRO::Control()
{
  if (true == readData())
  {
    currentTime = this->get_clock()->now();
    double dt = 0.0;
    if (lastTime.nanoseconds() != 0) {
      dt = (currentTime - lastTime).seconds();
    }

    lastTime = currentTime;
    publisherOdom(dt);
    RCLCPP_INFO(this->get_logger(), "dt:%f", dt);
    publisherVoltage();
  }
}

AGV_PRO::AGV_PRO(std::string node_name):rclcpp::Node(node_name)
{
  this->declare_parameter<std::string>("port_name","/dev/agvpro_controller");
  this->declare_parameter<std::string>("odometry.frame_id", "odom");
  this->declare_parameter<std::string>("odometry.child_frame_id", "base_footprint");
  this->declare_parameter<std::string>("imu.frame_id", "imu_link");
  this->declare_parameter<std::string>("namespace", "");

  this->get_parameter_or<std::string>("port_name",device_name_,std::string("/dev/agvpro_controller"));
  this->get_parameter_or<std::string>("odometry.frame_id",frame_id_of_odometry_,std::string("odom"));
  this->get_parameter_or<std::string>("odometry.child_frame_id",child_frame_id_of_odometry_,std::string("base_footprint"));
  this->get_parameter_or<std::string>("imu.frame_id",frame_id_of_imu_,std::string("imu_link"));        
  this->get_parameter_or<std::string>("namespace",name_space_,std::string(""));

  if (name_space_ != "") {
    frame_id_of_odometry_ = name_space_ + "/" + frame_id_of_odometry_;
    child_frame_id_of_odometry_ = name_space_ + "/" + child_frame_id_of_odometry_;
    frame_id_of_imu_ = name_space_ + "/" + frame_id_of_imu_;
  }

  odomBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  pub_imu =  this->create_publisher<sensor_msgs::msg::Imu>("imu", 20);
  pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);
  pub_voltage = create_publisher<std_msgs::msg::Float32>("voltage", 10);
  cmd_sub = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&AGV_PRO::cmdCallback, this, std::placeholders::_1));

  lastTime = this->get_clock()->now();
      
  drivers::serial_driver::SerialPortConfig config(
    1000000,
    drivers::serial_driver::FlowControl::NONE,
    drivers::serial_driver::Parity::NONE,
    drivers::serial_driver::StopBits::ONE
  );

  try{
    io_context_ = std::make_shared<drivers::common::IoContext>(1);
    serial_driver_ = std::make_shared<drivers::serial_driver::SerialDriver>(*io_context_);
    serial_driver_->init_port(device_name_, config);
    serial_driver_->port()->open();
    
    RCLCPP_INFO(this->get_logger(), "Serial port initialized successfully");
    RCLCPP_INFO(this->get_logger(), "Using device: %s", serial_driver_->port().get()->device_name().c_str());
    RCLCPP_INFO(this->get_logger(), "Baud_rate: %d", config.get_baud_rate());

    AGV_PRO::set_auto_report();
  }
  catch (const std::exception &ex){
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port: %s", ex.what());
    return;
  }
  
  control_timer_ = this->create_wall_timer(
  std::chrono::milliseconds(20),
  std::bind(&AGV_PRO::Control, this)
  );

}

AGV_PRO::~AGV_PRO()
{ 
  std::array<uint8_t, 14> buf = {
    0xFE, 0xFE, 0x0b, 0x22, 
    0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
  };

  uint16_t crc = crc16_ibm(buf.data(), 12);
  buf[12] = (crc >> 8) & 0xff;
  buf[13] = crc & 0xff;

  std::vector<uint8_t> data_vec(buf.begin(), buf.end());

  auto port = serial_driver_->port();

  try
  {
    port->send(data_vec);
  }
  catch(const std::exception &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "Error Transmiting from serial port:%s",ex.what());
  }

  serial_driver_->port()->close();
  RCLCPP_INFO(this->get_logger(),"Shutting down");
}