#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "costmap_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>

class CostmapNode : public rclcpp::Node {
public:
    CostmapNode(); 

private:
    void processLidarData(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    std::vector<int8_t> flattenGridTo1D() const;
    void expandObstacles(int inflation_radius);

    robot::CostmapCore costmap_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    static constexpr double GRID_RESOLUTION = 0.2;  
    static constexpr int SIZE_OF_MAP = 200;        
    static constexpr int MAX_OBSTACLE_COST = 100;   
    static constexpr int INFLATION_RADIUS_CELLS = 6; 
    static constexpr int GRID_ORIGIN = SIZE_OF_MAP / 2; 

    std::vector<std::vector<int>> occupancy_grid_;
};

#endif
