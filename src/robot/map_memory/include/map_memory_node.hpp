#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "map_memory_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <nav_msgs/msg/odometry.hpp>

class MapMemoryNode : public rclcpp::Node {
    public:
        MapMemoryNode();
        void handleOdometryUpdate(const nav_msgs::msg::Odometry::SharedPtr msg);
        void updateGlobalMap();
        void integrateCostmapIntoGlobalMap();
        void publishGlobalMap();

    private:
        robot::MapMemoryCore map_memory_core_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscriber_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
        rclcpp::TimerBase::SharedPtr update_timer_;
        static constexpr double RESOLUTION = 0.3;
        static constexpr int SIZE_OF_MAP = 100;
        nav_msgs::msg::OccupancyGrid global_map_;
        nav_msgs::msg::OccupancyGrid latest_costmap_;
        double previous_x_ = 0.0;
        double previous_y_ = 0.0;
        double previous_yaw_ = 0.0;
        const double update_distance_threshold_ = 3.0;
        bool is_costmap_updated_ = false;
        bool should_update_map_ = false;
        bool is_map_initialized_ = false;
};

#endif
