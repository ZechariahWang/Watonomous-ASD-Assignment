#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <optional>
#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    robot::ControlCore control_system_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    nav_msgs::msg::Path::SharedPtr active_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odometry_;
    double lookahead_dist_;
    double goal_reach_tolerance_;
    double forward_speed_;
    std::optional<geometry_msgs::msg::PoseStamped> locateLookaheadTarget();
    geometry_msgs::msg::Twist generateVelocityCommand(const geometry_msgs::msg::PoseStamped &target_pose);
    void executeControlLoop();
};

#endif
