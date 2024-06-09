#include "rclcpp/rclcpp.hpp"
#include "angles/angles.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rmw/qos_profiles.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/detail/empty__struct.hpp"
#include "tf2/impl/utils.h"
#include "attach_shelf/srv/go_to_loading.hpp"
#include "std_msgs/msg/empty.hpp"

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <thread>

template <class T> inline int sgn(T v) { return (v > T(0)) - (v < T(0)); }

using namespace std::chrono_literals;
using GoToLoading = attach_shelf::srv::GoToLoading;

class PreApproach : public rclcpp::Node {
public:
  PreApproach()
      : Node("robot_shelf_pre_approach_V2_node"), front_dist_(float INFINITY) {
    using namespace std::placeholders;

    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description =
        "Distance (in meters) to the obstacle at which the robot will stop.";
    this->declare_parameter<float>("obstacle", 0.3, param_desc);

    param_desc.description =
        "Number of degrees for the rotation of the robot after stopping.";
    this->declare_parameter<int16_t>("degrees", 0, param_desc);

    param_desc.description =
        "Boolean parameter to specify if the robot will do the finale approach or not.";
    this->declare_parameter<bool>("final_approach", false, param_desc);

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

    client_cbg_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    client_ = this->create_client<GoToLoading>("approach_shelf", 
        rmw_qos_profile_services_default, client_cbg_);

    elevator_up_pub_ = this->create_publisher<std_msgs::msg::Empty>(
        "elevator_up", 1);
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
    this->get_parameter("final_approach", final_approach_);
    RCLCPP_INFO(
        this->get_logger(),
        "The robot is going to move forward until it detects an obstacle in "
        "front of it at %.2f m, it will then rotate for %d degrees and final approach %s",
        obs_param_, deg_param_, final_approach_? "true": "false");

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

    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Client interrupted while waiting for service. Terminating...");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service Unavailable. Waiting for Service...");
    }
    RCLCPP_INFO(this->get_logger(), "Pre-approach mission done.");

    auto request = std::make_shared<GoToLoading::Request>();
    request->attach_to_shelf = final_approach_;

    auto result_future = client_->async_send_request(
        request, std::bind(&PreApproach::response_callback, this,
                           std::placeholders::_1));
    
    
  }

  void response_callback(rclcpp::Client<GoToLoading>::SharedFuture future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
        auto result = future.get();
        if (result->complete) {
            std::this_thread::sleep_for(50ms);
            std_msgs::msg::Empty msg;
            elevator_up_pub_->publish(msg);

            RCLCPP_INFO(this->get_logger(), "The shelf is now attached to the robot.");
        }
        else {
            RCLCPP_INFO(this->get_logger(), "The final approach failed.");
        }
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
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
  rclcpp::CallbackGroup::SharedPtr client_cbg_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<GoToLoading>::SharedPtr client_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr elevator_up_pub_; 

  float obs_param_;
  int16_t deg_param_;
  bool final_approach_;

  float front_dist_;
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