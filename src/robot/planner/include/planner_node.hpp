#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "planner_core.hpp"

#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <cstddef>

struct CellIndex {
    int x;
    int y;

    CellIndex(int xx, int yy) : x(xx), y(yy) {}
    CellIndex() : x(0), y(0) {}

    bool operator==(const CellIndex &other) const { return (x == other.x && y == other.y); }
    bool operator!=(const CellIndex &other) const { return !(*this == other); }
};

struct CellIndexHash {
    std::size_t operator()(const CellIndex &idx) const { return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1); }
};

struct AStarNode {
    CellIndex index;    
    double f_score;     
    double g_score;     
    double h_score;     
    CellIndex parent;   

    AStarNode()
        : index(0, 0), f_score(INFINITY), g_score(INFINITY), h_score(INFINITY), parent(0, 0) {}
    AStarNode(CellIndex idx, double f, double g, double h, CellIndex p)
        : index(idx), f_score(f), g_score(g), h_score(h), parent(p) {}
};

struct CompareF { bool operator()(const AStarNode &a, const AStarNode &b) { return a.f_score > b.f_score; } };

enum class PlannerState {
    AWAITING_GOAL,
    PLANNING_PATH
};

class PlannerNode : public rclcpp::Node {

  public:

      PlannerNode();
      void handleMapUpdate(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
      void handleGoalUpdate(const geometry_msgs::msg::PointStamped::SharedPtr msg);
      void handleOdometryUpdate(const nav_msgs::msg::Odometry::SharedPtr msg);
      void onTimerEvent();
      bool hasGoalBeenReached();
      void computePath();
      void publishPath(const std::vector<geometry_msgs::msg::PoseStamped> &path);
      void publishEmptyPath();
      double movementCost(const CellIndex &current, const CellIndex &neighbor);
      geometry_msgs::msg::PoseStamped createPose(const CellIndex &index);

  private:

      robot::PlannerCore planning_core_;
      PlannerState state_ = PlannerState::AWAITING_GOAL;
      rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
      rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_subscription_;
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
      rclcpp::TimerBase::SharedPtr planning_timer_;
      nav_msgs::msg::OccupancyGrid current_map_;
      geometry_msgs::msg::PointStamped goal_;
      geometry_msgs::msg::Pose robot_pose_;
      bool goal_set_ = false;
      const std::string FRAME_ID = "sim_world";
      CellIndex mapToCell(const geometry_msgs::msg::Point &point);  
      geometry_msgs::msg::Point cellToWorld(const CellIndex &cell); 
      double calculateHeuristic(const CellIndex &start, const CellIndex &target);
      std::vector<CellIndex> findNeighbors(const CellIndex &current);
      std::vector<geometry_msgs::msg::PoseStamped> retracePath(const AStarNode &goal_node);
      double cellCost(const CellIndex &cell);
      int OBSTACLE_THRESHOLD = 15;
      std::unordered_map<CellIndex, AStarNode, CellIndexHash> node_registry_;
};

#endif
