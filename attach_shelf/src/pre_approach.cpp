#include "angles/angles.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/impl/utils.h"
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <thread>

template <class T> inline int sgn(T v) { return (v > T(0)) - (v < T(0)); }

using namespace std::chrono_literals;

class PreApproach : public rclcpp::Node {
public:
  PreApproach()
      : Node("robot_shelf_pre_approach_node"), front_dist_(float INFINITY) {
    using namespace std::placeholders;

    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description =
        "Distance (in meters) to the obstacle at which the robot will stop.";
    this->declare_parameter<float>("obstacle", 0.3, param_desc);

    param_desc.description =
        "Number of degrees for the rotation of the robot after stopping.";
    this->declare_parameter<int16_t>("degrees", 0, param_desc);

    scan_sub_cbg_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options_1;
    options_1.callback_group = scan_sub_cbg_;

    rclcpp::QoS qos_profile_scan_sub(10);
    qos_profile_scan_sub.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", qos_profile_scan_sub,
        std::bind(&PreApproach::scan_callback, this, _1), options_1);

    odom_sub_cbg_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options_2;
    options_2.callback_group = odom_sub_cbg_;

    rclcpp::QoS qos_profile_odom_sub(10);
    qos_profile_odom_sub.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", qos_profile_odom_sub,
        std::bind(&PreApproach::odom_callback, this, _1), options_2);

    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 1);

    timer_cbg_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = this->create_wall_timer(
        100ms, std::bind(&PreApproach::timer_callback, this), timer_cbg_);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    size_t front_ray_index = msg->ranges.size() / 2;
    front_dist_ = msg->ranges[front_ray_index];
  }

  void timer_callback() {
    timer_->cancel();
    this->get_parameter("obstacle", obs_param_);
    this->get_parameter("degrees", deg_param_);
    RCLCPP_INFO(
        this->get_logger(),
        "The robot is going to move forward until it detects an obstacle in "
        "front of it at %.2f m and it will then rotate for %d degrees",
        obs_param_, deg_param_);

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.4;

    while (front_dist_ > obs_param_) {
      cmd_vel_pub_->publish(cmd_vel);
      std::this_thread::sleep_for(50ms);
    }

    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = sgn<int>(deg_param_) * 0.4;

    auto target_rad =
        angles::normalize_angle(yaw_ + angles::from_degrees(deg_param_));

    while (std::abs(target_rad - yaw_) > 0.0175) {
      cmd_vel_pub_->publish(cmd_vel);
      std::this_thread::sleep_for(50ms);
    }

    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel);
    RCLCPP_INFO(this->get_logger(), "Pre-approach mission done, the robot has stopped");
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    yaw_ = tf2::impl::getYaw(q);
  }

  rclcpp::CallbackGroup::SharedPtr scan_sub_cbg_;
  rclcpp::CallbackGroup::SharedPtr odom_sub_cbg_;
  rclcpp::CallbackGroup::SharedPtr timer_cbg_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  float front_dist_;
  float obs_param_;
  int16_t deg_param_;
  tf2Scalar yaw_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<PreApproach> pre_approach_node =
      std::make_shared<PreApproach>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(pre_approach_node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}