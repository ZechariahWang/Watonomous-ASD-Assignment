#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
    occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
    lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar", 10,
        std::bind(&CostmapNode::processLidarData, this, std::placeholders::_1));
    status_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        [this]() { RCLCPP_INFO(this->get_logger(), "CostmapNode is operational"); });
}

std::vector<int8_t> CostmapNode::flattenGridTo1D() const {
    std::vector<int8_t> flattened(SIZE_OF_MAP * SIZE_OF_MAP);
    for (int y = 0; y < SIZE_OF_MAP; ++y) {
        for (int x = 0; x < SIZE_OF_MAP; ++x) {
            flattened[y * SIZE_OF_MAP + x] = static_cast<int8_t>(occupancy_grid_[x][y]);
        }
    }
    return flattened;
}

void CostmapNode::processLidarData(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    if (!scan) {
        RCLCPP_ERROR(this->get_logger(), "Error: No LiDAR data received");
        return;
    }

    int grid_size = SIZE_OF_MAP;
    occupancy_grid_.assign(grid_size, std::vector<int>(grid_size, 0));

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];

        if (range >= scan->range_min && range <= scan->range_max) {
            int x_grid = GRID_ORIGIN + static_cast<int>(range * std::cos(angle) / GRID_RESOLUTION);
            int y_grid = GRID_ORIGIN + static_cast<int>(range * std::sin(angle) / GRID_RESOLUTION);
            if (x_grid >= 0 && x_grid < SIZE_OF_MAP && y_grid >= 0 && y_grid < SIZE_OF_MAP) {
                occupancy_grid_[x_grid][y_grid] = MAX_OBSTACLE_COST;
            }
        }
    }

    expandObstacles(INFLATION_RADIUS_CELLS);

    const std_msgs::msg::Header& header = scan->header;
    nav_msgs::msg::OccupancyGrid message;
    message.header = header;
    message.info.resolution = GRID_RESOLUTION;
    message.info.width = SIZE_OF_MAP;
    message.info.height = SIZE_OF_MAP;
    message.info.origin.position.x = -GRID_ORIGIN * GRID_RESOLUTION;
    message.info.origin.position.y = -GRID_ORIGIN * GRID_RESOLUTION;
    message.data = flattenGridTo1D();
    occupancy_grid_publisher_->publish(message);
}

void CostmapNode::expandObstacles(int inflation_radius) {
    const int radius_squared = inflation_radius * inflation_radius;
    for (int y = 0; y < SIZE_OF_MAP; ++y) {
        for (int x = 0; x < SIZE_OF_MAP; ++x) {
            if (occupancy_grid_[x][y] != MAX_OBSTACLE_COST) continue;

            int min_x = std::max(0, x - inflation_radius);
            int max_x = std::min(SIZE_OF_MAP - 1, x + inflation_radius);
            int min_y = std::max(0, y - inflation_radius);
            int max_y = std::min(SIZE_OF_MAP - 1, y + inflation_radius);

            for (int ny = min_y; ny <= max_y; ++ny) {
                for (int nx = min_x; nx <= max_x; ++nx) {
                    int dx = nx - x;
                    int dy = ny - y;

                    if (dx * dx + dy * dy <= radius_squared) {
                        int cost = static_cast<int>(
                            MAX_OBSTACLE_COST * (1.0 - std::sqrt(dx * dx + dy * dy) / inflation_radius));
                        occupancy_grid_[nx][ny] = std::max(occupancy_grid_[nx][ny], cost);
                    }
                }
            }
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapNode>());
    rclcpp::shutdown();
    return 0;
}
