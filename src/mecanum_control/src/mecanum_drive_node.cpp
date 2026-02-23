// Copyright (c) 2025 Tarik Kahraman
// SPDX-License-Identifier: MIT

#include <chrono>
#include <memory>
#include <string>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "mecanum_control/mecanum_kinematics.hpp"

using namespace std::chrono_literals;

namespace mecanum_control
{

/**
 * @brief Mecanum Drive Node (C++)
 *
 * /cmd_vel → Ters kinematik → Tekerlek hız komutları
 * Tekerlek encoder → İleri kinematik → /odom
 *
 * Subscribed Topics:
 *   /cmd_vel          (geometry_msgs/Twist)
 *   /joint_states     (sensor_msgs/JointState)
 *
 * Published Topics:
 *   /wheel_speeds     (std_msgs/Float64MultiArray) [fl, fr, rl, rr] rad/s
 *   /odom             (nav_msgs/Odometry)
 */
class MecanumDriveNode : public rclcpp::Node
{
public:
  MecanumDriveNode()
  : Node("mecanum_drive_node"),
    x_(0.0), y_(0.0), theta_(0.0),
    last_time_(this->get_clock()->now())
  {
    // Parametreler
    this->declare_parameter("wheel_radius", 0.075);
    this->declare_parameter("lx", 0.25);
    this->declare_parameter("ly", 0.35);
    this->declare_parameter("max_wheel_speed", 15.0);
    this->declare_parameter("publish_rate", 50.0);

    const double r = this->get_parameter("wheel_radius").as_double();
    const double lx = this->get_parameter("lx").as_double();
    const double ly = this->get_parameter("ly").as_double();
    max_speed_ = this->get_parameter("max_wheel_speed").as_double();

    kinematics_ = std::make_unique<MecanumKinematics>(r, lx, ly);

    RCLCPP_INFO(
      this->get_logger(),
      "MecanumDriveNode başlatıldı: r=%.3f lx=%.3f ly=%.3f", r, lx, ly);

    // Subscriber: cmd_vel
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&MecanumDriveNode::cmdVelCallback, this, std::placeholders::_1)
    );

    // Subscriber: joint_states (encoder geri bildirimi)
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&MecanumDriveNode::jointStateCallback, this, std::placeholders::_1)
    );

    // Publisher: tekerlek hız komutları
    wheel_speed_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "wheel_speeds", 10
    );

    // Publisher: odometri
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // Komut zaman aşımı kontrolü (500ms'de komut gelmezse dur)
    watchdog_timer_ = this->create_wall_timer(
      100ms,
      std::bind(&MecanumDriveNode::watchdogCallback, this)
    );

    last_cmd_time_ = this->get_clock()->now();
  }

private:
  // ------------------------------------------------------------------ //

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_time_ = this->get_clock()->now();

    // Ters kinematik
    auto speeds = kinematics_->inverse(msg->linear.x, msg->linear.y, msg->angular.z);
    speeds = kinematics_->normalize(speeds, max_speed_);

    // Yayınla
    auto out = std_msgs::msg::Float64MultiArray();
    out.data = std::vector<double>(speeds.begin(), speeds.end());
    wheel_speed_pub_->publish(out);

    RCLCPP_DEBUG(
      this->get_logger(),
      "cmd_vel → wheels: FL=%.2f FR=%.2f RL=%.2f RR=%.2f",
      speeds[0], speeds[1], speeds[2], speeds[3]);
  }

  // ------------------------------------------------------------------ //

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Tekerlek hızlarını joint_states'den al
    // Sıra: front_left, front_right, rear_left, rear_right
    if (msg->velocity.size() < 4) {return;}

    std::array<double, 4> ws = {
      msg->velocity[0], msg->velocity[1],
      msg->velocity[2], msg->velocity[3]
    };

    auto body = kinematics_->forward(ws);

    // Odometri entegrasyonu (basit Euler)
    auto now = this->get_clock()->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    if (dt <= 0.0 || dt > 1.0) {return;}

    // Dünya çerçevesinde konum güncelle
    double delta_x = (body.vx * std::cos(theta_) - body.vy * std::sin(theta_)) * dt;
    double delta_y = (body.vx * std::sin(theta_) + body.vy * std::cos(theta_)) * dt;
    x_ += delta_x;
    y_ += delta_y;
    theta_ += body.omega * dt;

    // Theta normalize (-pi, pi)
    while (theta_ > M_PI) {theta_ -= 2.0 * M_PI;}
    while (theta_ < -M_PI) {theta_ += 2.0 * M_PI;}

    publishOdom(body, now);
  }

  // ------------------------------------------------------------------ //

  void publishOdom(
    const MecanumKinematics::BodyVelocity & vel,
    const rclcpp::Time & stamp)
  {
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    // Konum
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    // Quaternion (sadece yaw)
    odom.pose.pose.orientation.z = std::sin(theta_ / 2.0);
    odom.pose.pose.orientation.w = std::cos(theta_ / 2.0);

    // Hız
    odom.twist.twist.linear.x = vel.vx;
    odom.twist.twist.linear.y = vel.vy;
    odom.twist.twist.angular.z = vel.omega;

    // Kovaryans (basit sabit değerler)
    odom.pose.covariance[0] = 0.01;
    odom.pose.covariance[7] = 0.01;
    odom.pose.covariance[35] = 0.01;
    odom.twist.covariance[0] = 0.01;
    odom.twist.covariance[7] = 0.01;
    odom.twist.covariance[35] = 0.01;

    odom_pub_->publish(odom);
  }

  // ------------------------------------------------------------------ //

  void watchdogCallback()
  {
    auto now = this->get_clock()->now();
    double elapsed = (now - last_cmd_time_).seconds();

    // 500ms'de komut gelmediyse robotu durdur
    if (elapsed > 0.5) {
      auto zero = std_msgs::msg::Float64MultiArray();
      zero.data = {0.0, 0.0, 0.0, 0.0};
      wheel_speed_pub_->publish(zero);
    }
  }

  // ------------------------------------------------------------------ //

  std::unique_ptr<MecanumKinematics> kinematics_;
  double max_speed_;

  // Odometri durumu
  double x_, y_, theta_;
  rclcpp::Time last_time_;
  rclcpp::Time last_cmd_time_;

  // ROS arayüzleri
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_speed_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace mecanum_control

// ------------------------------------------------------------------ //

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mecanum_control::MecanumDriveNode>());
  rclcpp::shutdown();
  return 0;
}
