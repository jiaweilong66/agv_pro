// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \brief  Simple model controller that uses a twist message to move an entity on the xy plane.
 *
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * \date  29 July 2013
 */

#include <gazebo/common/Events.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_plugins/gazebo_ros_planar_move.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>

namespace gazebo_plugins
{
class GazeboRosPlanarMovePrivate
{
public:
  void OnUpdate(const gazebo::common::UpdateInfo & _info);
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);
  void UpdateOdometry(const gazebo::common::Time & _current_time);
  void PublishOdometryTf(const gazebo::common::Time & _current_time);

  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  geometry_msgs::msg::Twist target_cmd_vel_;
  nav_msgs::msg::Odometry odom_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr model_;
  gazebo::event::ConnectionPtr update_connection_;
  std::mutex lock_;
  double update_period_;
  double publish_period_;
  gazebo::common::Time last_update_time_;
  gazebo::common::Time last_publish_time_;
  std::string odometry_frame_;
  std::string robot_base_frame_;
  bool publish_odom_;
  bool publish_odom_tf_;

  // New: scale factors
  double linear_multiplier_;
  double angular_multiplier_;
};

GazeboRosPlanarMove::GazeboRosPlanarMove()
: impl_(std::make_unique<GazeboRosPlanarMovePrivate>())
{
}

GazeboRosPlanarMove::~GazeboRosPlanarMove()
{
}

void GazeboRosPlanarMove::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;
  impl_->world_ = _model->GetWorld();
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  impl_->odometry_frame_ = _sdf->Get<std::string>("odometry_frame", "odom").first;
  impl_->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_footprint").first;

  auto update_rate = _sdf->Get<double>("update_rate", 20.0).first;
  impl_->update_period_ = (update_rate > 0.0) ? 1.0 / update_rate : 0.0;
  impl_->last_update_time_ = impl_->world_->SimTime();

  auto publish_rate = _sdf->Get<double>("publish_rate", 20.0).first;
  impl_->publish_period_ = (publish_rate > 0.0) ? 1.0 / publish_rate : 0.0;
  impl_->last_publish_time_ = impl_->world_->SimTime();

  // Load new scale parameters
  impl_->linear_multiplier_ = _sdf->Get<double>("linear_multiplier", 1.0).first;
  impl_->angular_multiplier_ = _sdf->Get<double>("angular_multiplier", 1.0).first;

  impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
    std::bind(&GazeboRosPlanarMovePrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(), "Subscribed to [%s]",
    impl_->cmd_vel_sub_->get_topic_name());

  impl_->publish_odom_ = _sdf->Get<bool>("publish_odom", true).first;
  if (impl_->publish_odom_) {
    impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
      "odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Advertise odometry on [%s]",
      impl_->odometry_pub_->get_topic_name());
  }

  impl_->publish_odom_tf_ = _sdf->Get<bool>("publish_odom_tf", true).first;
  if (impl_->publish_odom_tf_) {
    impl_->transform_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(),
      "Publishing odom transforms between [%s] and [%s]",
      impl_->odometry_frame_.c_str(), impl_->robot_base_frame_.c_str());
  }

  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosPlanarMovePrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosPlanarMove::Reset()
{
  impl_->last_update_time_ = impl_->world_->SimTime();
  impl_->target_cmd_vel_.linear.x = 0;
  impl_->target_cmd_vel_.linear.y = 0;
  impl_->target_cmd_vel_.angular.z = 0;
}

void GazeboRosPlanarMovePrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();
  std::lock_guard<std::mutex> scoped_lock(lock_);
  if (seconds_since_last_update >= update_period_) {
    ignition::math::Pose3d pose = model_->WorldPose();
    float yaw = static_cast<float>(pose.Rot().Yaw());

    // Apply scaling to commanded velocities
    double vx = target_cmd_vel_.linear.x * linear_multiplier_;
    double vy = target_cmd_vel_.linear.y * linear_multiplier_;
    double wz = target_cmd_vel_.angular.z * angular_multiplier_;

    model_->SetLinearVel(
      ignition::math::Vector3d(
        vx * cosf(yaw) - vy * sinf(yaw),
        vy * cosf(yaw) + vx * sinf(yaw),
        0));
    model_->SetAngularVel(
      ignition::math::Vector3d(0, 0, wz));

    last_update_time_ = _info.simTime;
  }

  if (publish_odom_ || publish_odom_tf_) {
    double seconds_since_last_publish = (_info.simTime - last_publish_time_).Double();
    if (seconds_since_last_publish < publish_period_) {
      return;
    }
    UpdateOdometry(_info.simTime);
    if (publish_odom_) {
      odometry_pub_->publish(odom_);
    }
    if (publish_odom_tf_) {
      PublishOdometryTf(_info.simTime);
    }
    last_publish_time_ = _info.simTime;
  }
}

void GazeboRosPlanarMovePrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  target_cmd_vel_ = *_msg;
}

void GazeboRosPlanarMovePrivate::UpdateOdometry(const gazebo::common::Time & _current_time)
{
  auto pose = model_->WorldPose();
  odom_.pose.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(pose);
  odom_.twist.twist.angular.z = model_->WorldAngularVel().Z();
  auto linear = model_->WorldLinearVel();
  auto yaw = static_cast<float>(pose.Rot().Yaw());
  odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
  odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
  odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
}

void GazeboRosPlanarMovePrivate::PublishOdometryTf(const gazebo::common::Time & _current_time)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
  msg.header.frame_id = odometry_frame_;
  msg.child_frame_id = robot_base_frame_;
  msg.transform = gazebo_ros::Convert<geometry_msgs::msg::Transform>(odom_.pose.pose);
  transform_broadcaster_->sendTransform(msg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosPlanarMove)
}  // namespace gazebo_plugins
