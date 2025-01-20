#include "map_memory_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_core_(robot::MapMemoryCore(this->get_logger())), previous_x_(0.0), previous_y_(0.0), update_distance_threshold_(3.0) {
    costmap_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            latest_costmap_ = *msg;
            is_costmap_updated_ = true;

            bool is_empty = std::all_of(msg->data.begin(), msg->data.end(), [](int8_t value) { return value == 0; });
            if (should_update_map_ && !is_empty) {
                updateGlobalMap();
            }
        });

    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            const auto &orientation = msg->pose.pose.orientation;

            tf2::Quaternion quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
            tf2::Matrix3x3 matrix(quaternion);
            double roll, pitch, yaw;
            matrix.getRPY(roll, pitch, yaw);

            double x = msg->pose.pose.position.x;
            double y = msg->pose.pose.position.y;

            double distance_moved = std::sqrt(
                std::pow(x - previous_x_, 2) + std::pow(y - previous_y_, 2));

            if (distance_moved >= update_distance_threshold_) {
                previous_x_ = x;
                previous_y_ = y;
                should_update_map_ = true;
            }
        });

    map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    global_map_.data.resize(SIZE_OF_MAP * SIZE_OF_MAP, 0);
    is_map_initialized_ = true;
    update_timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() { publishGlobalMap(); });
}

void MapMemoryNode::handleOdometryUpdate(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    const auto &orientation = msg->pose.pose.orientation;
    tf2::Quaternion quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
    tf2::Matrix3x3 matrix(quaternion);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);

    double distance_moved = std::sqrt(std::pow(x - previous_x_, 2) + std::pow(y - previous_y_, 2));

    if (distance_moved >= update_distance_threshold_ || is_map_initialized_) {
        previous_x_ = x;
        previous_y_ = y;
        previous_yaw_ = yaw;
        is_map_initialized_ = false;
        should_update_map_ = true;
    }
}

void MapMemoryNode::publishGlobalMap() {
    global_map_.header.stamp = this->now();
    global_map_.header.frame_id = "sim_world";
    map_publisher_->publish(global_map_);
}

void MapMemoryNode::updateGlobalMap() {
  integrateCostmapIntoGlobalMap();

  global_map_.info.resolution = RESOLUTION;
  global_map_.info.width = SIZE_OF_MAP;
  global_map_.info.height = SIZE_OF_MAP;

  const double half_size_res = -SIZE_OF_MAP / 2.0 * RESOLUTION;
  global_map_.info.origin.position.x = half_size_res;
  global_map_.info.origin.position.y = half_size_res;

  should_update_map_ = false;
}

void MapMemoryNode::integrateCostmapIntoGlobalMap() {
  const std::vector<int8_t> &costmap_data = latest_costmap_.data;
  std::vector<int8_t> &global_map_data = global_map_.data;

  double cos_yaw = std::cos(previous_yaw_);
  double sin_yaw = std::sin(previous_yaw_);

  int local_size = latest_costmap_.info.width;
  double local_origin_x = latest_costmap_.info.origin.position.x;
  double local_origin_y = latest_costmap_.info.origin.position.y;
  double local_resolution = latest_costmap_.info.resolution;

  double global_origin_offset = SIZE_OF_MAP / -2 * RESOLUTION;
  double resolution_inv = 1.0 / RESOLUTION;

  for (int i = 0; i < local_size; ++i) {
      double local_y = local_origin_y + (i + 0.5) * local_resolution;
      double transformed_y = sin_yaw * local_y;
      double rotated_y = cos_yaw * local_y;

      for (int j = 0; j < local_size; ++j) {
          int occupancy_value = costmap_data[i * local_size + j];
          if (occupancy_value < 0) { continue; }

          double local_x = local_origin_x + (j + 0.5) * local_resolution;
          double global_x = previous_x_ + (cos_yaw * local_x - transformed_y);
          double global_y = previous_y_ + (sin_yaw * local_x + rotated_y);

          int global_map_x = static_cast<int>((global_x - global_origin_offset) * resolution_inv);
          int global_map_y = static_cast<int>((global_y - global_origin_offset) * resolution_inv);

          if (global_map_x >= 0 && global_map_x < SIZE_OF_MAP && global_map_y >= 0 && global_map_y < SIZE_OF_MAP) {
              int map_index = global_map_y * SIZE_OF_MAP + global_map_x;
              int8_t &global_cell_value = global_map_data[map_index];
              global_cell_value = std::max(global_cell_value < 0 ? 0 : global_cell_value, occupancy_value);
          }
      }
  }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMemoryNode>());
    rclcpp::shutdown();
    return 0;
}
