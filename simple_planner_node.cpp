#include <unordered_map>
#include <utility>
#include <memory>
#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

struct GridNode {
    int x, y;
    float g_cost, h_cost;
    float closest_object_distance;
    std::shared_ptr<GridNode> parent;

    bool operator>(const GridNode& other) const {
        return (g_cost + h_cost) > (other.g_cost + other.h_cost);
    }
};

struct GridNodeHash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        auto h1 = std::hash<int>{}(p.first);
        auto h2 = std::hash<int>{}(p.second);
        return h1 ^ (h2 << 1); // Combine hashes
    }
};

class SimplePlannerNode : public rclcpp::Node {
public:
    SimplePlannerNode() : Node("simple_planner_node") {
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/grid_map", 10, std::bind(&SimplePlannerNode::gridMapCallback, this, std::placeholders::_1));

        goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&SimplePlannerNode::goalPoseCallback, this, std::placeholders::_1));

        initialpose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, std::bind(&SimplePlannerNode::initialposeCallback, this, std::placeholders::_1));

        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/computed_path", 10);

        RCLCPP_INFO(this->get_logger(), "SimplePlannerNode initialized.");
    }

private:
    void gridMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Received grid map with resolution: %f", map_.info.resolution);

        // Compute the distance to the nearest obstacle for each cell
        computeObstacleDistances();
    }

    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received goal pose at (%.2f, %.2f)",
            msg->pose.position.x, msg->pose.position.y);

        if (map_.data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Map data is not available. Cannot compute path.");
            return;
        }

        if (!initial_pose_set_) {
            RCLCPP_WARN(this->get_logger(), "Initial pose is not set. Cannot compute path.");
            return;
        }

        int goal_x = static_cast<int>(msg->pose.position.x / map_.info.resolution);
        int goal_y = static_cast<int>(msg->pose.position.y / map_.info.resolution);
        int start_x = static_cast<int>(initial_pose_.position.x / map_.info.resolution);
        int start_y = static_cast<int>(initial_pose_.position.y / map_.info.resolution);

        computePath(start_x, start_y, goal_x, goal_y);
    }

    void initialposeCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        initial_pose_ = msg->pose.pose;
        initial_pose_set_ = true;
        RCLCPP_INFO(this->get_logger(), "Updated initial pose to: [x: %f, y: %f]", initial_pose_.position.x, initial_pose_.position.y);
    }

    void computeObstacleDistances() {
        obstacle_distances_.resize(map_.info.height, std::vector<float>(map_.info.width, std::numeric_limits<float>::max()));

        for (int y = 0; y < static_cast<int>(map_.info.height); ++y) {
            for (int x = 0; x < static_cast<int>(map_.info.width); ++x) {
                if (map_.data[y * map_.info.width + x] == 100) {
                    obstacle_distances_[y][x] = 0.0f;
                }
            }
        }

        std::queue<std::pair<int, int>> q;
        for (int y = 0; y < static_cast<int>(map_.info.height); ++y) {
            for (int x = 0; x < static_cast<int>(map_.info.width); ++x) {
                if (map_.data[y * map_.info.width + x] == 100) {
                    q.push({ x, y });
                }
            }
        }

        std::vector<std::pair<int, int>> directions = { {-1, 0}, {1, 0}, {0, -1}, {0, 1} };
        while (!q.empty()) {
            auto [x, y] = q.front();
            q.pop();

            for (auto& dir : directions) {
                int nx = x + dir.first;
                int ny = y + dir.second;

                if (nx >= 0 && nx < static_cast<int>(map_.info.width) &&
                    ny >= 0 && ny < static_cast<int>(map_.info.height) &&
                    obstacle_distances_[ny][nx] > obstacle_distances_[y][x] + 1.0f) {
                    obstacle_distances_[ny][nx] = obstacle_distances_[y][x] + 1.0f;
                    q.push({ nx, ny });
                }
            }
        }
    }

    void computePath(int start_x, int start_y, int goal_x, int goal_y) {
        GridNode start_node{ start_x, start_y, 0.0f, static_cast<float>(std::abs(goal_x - start_x) + std::abs(goal_y - start_y)),
                             obstacle_distances_[start_y][start_x], nullptr };
        std::unordered_map<std::pair<int, int>, std::shared_ptr<GridNode>, GridNodeHash> all_nodes;
        all_nodes[{start_x, start_y}] = std::make_shared<GridNode>(start_node);

        std::priority_queue<std::shared_ptr<GridNode>, std::vector<std::shared_ptr<GridNode>>, std::greater<>> open_list;
        open_list.push(all_nodes[{start_x, start_y}]);

        std::vector<std::vector<bool>> closed_list(map_.info.height, std::vector<bool>(map_.info.width, false));

        std::vector<std::pair<int, int>> directions = { {-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {1, 1}, {-1, 1}, {1, -1} };

        while (!open_list.empty()) {
            auto current_node = open_list.top();
            open_list.pop();

            if (current_node->x == goal_x && current_node->y == goal_y) {
                RCLCPP_INFO(this->get_logger(), "Goal reached at (%d, %d)", current_node->x, current_node->y);
                auto path = reconstructPath(current_node);
                smoothPath(path);
                publishPath(path);
                return;
            }

            closed_list[current_node->y][current_node->x] = true;

            for (const auto& dir : directions) {
                int neighbor_x = current_node->x + dir.first;
                int neighbor_y = current_node->y + dir.second;

                if (neighbor_x < 0 || neighbor_x >= static_cast<int>(map_.info.width) ||
                    neighbor_y < 0 || neighbor_y >= static_cast<int>(map_.info.height) ||
                    closed_list[neighbor_y][neighbor_x] || map_.data[neighbor_y * map_.info.width + neighbor_x] == 100) {
                    continue; // Skip obstacles
                }

                size_t index = neighbor_y * map_.info.width + neighbor_x;


                float g_cost = current_node->g_cost + ((dir.first == 0 || dir.second == 0) ? 1.0f : std::sqrt(2.0f));
                float h_cost = std::abs(neighbor_x - goal_x) + std::abs(neighbor_y - goal_y); // Manhattan distance
                float penalty = 0.1f * obstacle_distances_[neighbor_y][neighbor_x]; // Penalty for proximity to obstacles
                float turn_penalty = (dir.first == 0 || dir.second == 0) ? 0.0f : 1.0f; // Penalize diagonal moves
                float total_cost = g_cost + h_cost * 3.0f + penalty * 0.1f + turn_penalty * 0.0001f ;

                auto neighbor_node = std::make_shared<GridNode>(GridNode{ neighbor_x, neighbor_y, g_cost, h_cost, penalty, current_node });

                if (!all_nodes.count({ neighbor_x, neighbor_y }) ||
                    total_cost < (all_nodes[{neighbor_x, neighbor_y}]->g_cost + all_nodes[{neighbor_x, neighbor_y}]->h_cost)) {
                    all_nodes[{neighbor_x, neighbor_y}] = neighbor_node;
                    open_list.push(neighbor_node);
                }
            }
        }

        RCLCPP_WARN(this->get_logger(), "No path found to the goal.");
    }

    std::vector<geometry_msgs::msg::PoseStamped> reconstructPath(std::shared_ptr<GridNode> current_node) {
        std::vector<geometry_msgs::msg::PoseStamped> path;
        while (current_node != nullptr) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = current_node->x * map_.info.resolution;
            pose.pose.position.y = current_node->y * map_.info.resolution;
            pose.pose.position.z = 0.0;
            path.push_back(pose);
            current_node = current_node->parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    void smoothPath(std::vector<geometry_msgs::msg::PoseStamped>& path) {
        if (path.size() < 3) return;

        std::vector<geometry_msgs::msg::PoseStamped> smoothed_path;
        smoothed_path.push_back(path[0]);

        for (size_t i = 1; i < path.size() - 1; ++i) {
            float dx1 = path[i].pose.position.x - path[i - 1].pose.position.x;
            float dy1 = path[i].pose.position.y - path[i - 1].pose.position.y;
            float dx2 = path[i + 1].pose.position.x - path[i].pose.position.x;
            float dy2 = path[i + 1].pose.position.y - path[i].pose.position.y;

            if (std::sqrt(dx1 * dx1 + dy1 * dy1) + std::sqrt(dx2 * dx2 + dy2 * dy2) >
                std::sqrt((path[i + 1].pose.position.x - path[i - 1].pose.position.x) * (path[i + 1].pose.position.x - path[i - 1].pose.position.x) +
                    (path[i + 1].pose.position.y - path[i - 1].pose.position.y) * (path[i + 1].pose.position.y - path[i - 1].pose.position.y))) {
                smoothed_path.push_back(path[i]);
            }
        }

        smoothed_path.push_back(path.back());
        path = std::move(smoothed_path);
    }

    void publishPath(const std::vector<geometry_msgs::msg::PoseStamped>& path) {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = map_.header.frame_id;
        path_msg.header.stamp = this->now();
        path_msg.poses = path;
        path_publisher_->publish(path_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

    nav_msgs::msg::OccupancyGrid map_;
    geometry_msgs::msg::Pose initial_pose_;
    std::vector<std::vector<float>> obstacle_distances_;
    bool initial_pose_set_ = false;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePlannerNode>());
    rclcpp::shutdown();
    return 0;
}
