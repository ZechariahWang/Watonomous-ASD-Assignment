#include "control_node.hpp"

ControlNode::ControlNode() : Node("control"), control_system_(robot::ControlCore(this->get_logger())) {

    lookahead_dist_ = 1.0; 
    goal_reach_tolerance_ = 1.5; 
    forward_speed_ = 0.3; 

    path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr message)
        { active_path_ = message; });

    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr message)
        { robot_odometry_ = message; });

    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    control_loop_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), [this]()
        { executeControlLoop(); });
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::locateLookaheadTarget() {
    if (!active_path_ || active_path_->poses.empty() || !robot_odometry_) { return std::nullopt; }

    const auto &current_position = robot_odometry_->pose.pose.position;
    const geometry_msgs::msg::PoseStamped &goal_position = active_path_->poses.back();

    double distance_to_target = std::hypot(
        current_position.x - goal_position.pose.position.x,
        current_position.y - goal_position.pose.position.y);

    if (distance_to_target <= goal_reach_tolerance_) { return std::nullopt; }
    for (const auto &path_pose : active_path_->poses) {
        const auto &path_point = path_pose.pose.position;
        double point_distance = std::hypot(
            current_position.x - path_point.x,
            current_position.y - path_point.y);

        if (point_distance >= lookahead_dist_) { return path_pose; }
    }
    
    return std::nullopt;
}

void ControlNode::executeControlLoop() {
    if (!active_path_ || !robot_odometry_) { return; }

    auto target_point = locateLookaheadTarget();
    if (!target_point) {
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        velocity_publisher_->publish(stop_cmd); 
        return;
    }

    auto velocity_command = generateVelocityCommand(*target_point);
    velocity_publisher_->publish(velocity_command);
}

geometry_msgs::msg::Twist ControlNode::generateVelocityCommand(const geometry_msgs::msg::PoseStamped &target_pose) {
    geometry_msgs::msg::Twist command_velocity;

    const auto &robot_position = robot_odometry_->pose.pose.position;
    const auto &robot_orientation = robot_odometry_->pose.pose.orientation;

    double robot_x = robot_position.x;
    double robot_y = robot_position.y;

    double sin_heading = 2.0 * (robot_orientation.w * robot_orientation.z + robot_orientation.x * robot_orientation.y);
    double cos_heading = 1.0 - 2.0 * (robot_orientation.y * robot_orientation.y + robot_orientation.z * robot_orientation.z);
    double robot_heading = std::atan2(sin_heading, cos_heading);
    double delta_x = target_pose.pose.position.x - robot_x;
    double delta_y = target_pose.pose.position.y - robot_y;

    double target_distance = std::hypot(delta_x, delta_y);
    double target_heading = std::atan2(delta_y, delta_x);
    double angular_adjustment = std::fmod(target_heading - robot_heading + M_PI, 2.0 * M_PI) - M_PI;

    command_velocity.linear.x = target_distance;     
    command_velocity.angular.z = angular_adjustment;   

    return command_velocity;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}