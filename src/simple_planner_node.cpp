#include <unordered_map>
#include <utility>
#include <memory>
#include <queue>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream> // For debug output

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

struct GridNode {
    int x, y;
    float g_cost, h_cost;
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

    float getObstacleCost(int x, int y) {
        float penalty = 0.5f;
        int radius = 1.5; // Radius for checking around the cell
        for (int i = -radius; i <= radius; ++i) {
            for (int j = -radius; j <= radius; ++j) {
                int nx = x + i;
                int ny = y + j;
                if (nx >= 0 && nx < map_.info.width && ny >= 0 && ny < map_.info.height) {
                    if (map_.data[ny * map_.info.width + nx] == 100) {
                        // Increase penalty for cells close to obstacles
                        penalty += 0.5f;
                    }
                }
            }
        }
        return penalty;
    }

    bool isStraightLineClear(int start_x, int start_y, int goal_x, int goal_y) {
        int dx = std::abs(goal_x - start_x);
        int dy = std::abs(goal_y - start_y);
        int sx = (start_x < goal_x) ? 1 : -1;
        int sy = (start_y < goal_y) ? 1 : -1;

        int x = start_x;
        int y = start_y;
        int err = dx - dy;
        while (true) {
            if (x == goal_x && y == goal_y) {
                return true; // Reached goal
            }
            if (map_.data[y * map_.info.width + x] == 100) {
                return false; // An obstacle is found
            }
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x += sx;
            }
            if (e2 < dx) {
                err += dx;
                y += sy;
            }
        }
    }

    std::vector<geometry_msgs::msg::PoseStamped> smoothPath(const std::vector<geometry_msgs::msg::PoseStamped>& path, float epsilon) {
        std::vector<geometry_msgs::msg::PoseStamped> result;

        if (path.size() < 2) {
            return path; // No smoothing needed
        }

        std::function<void(size_t, size_t)> rdp;
        std::vector<size_t> indices_to_keep;

        rdp = [&](size_t start_idx, size_t end_idx) {
            float max_dist = 0.0f;
            size_t index = start_idx;

            for (size_t i = start_idx + 1; i < end_idx; ++i) {
                float dist = perpendicularDistance(path[start_idx], path[end_idx], path[i]);
                if (dist > max_dist) {
                    index = i;
                    max_dist = dist;
                }
            }

            if (max_dist > epsilon) {
                rdp(start_idx, index);
                indices_to_keep.push_back(index);
                rdp(index, end_idx);
            }
        };

        indices_to_keep.push_back(0);
        rdp(0, path.size() - 1);
        indices_to_keep.push_back(path.size() - 1);

        for (auto idx : indices_to_keep) {
            result.push_back(path[idx]);
        }

        return result;
    }

    float perpendicularDistance(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& end, const geometry_msgs::msg::PoseStamped& point) {
        float A = point.pose.position.y - start.pose.position.y;
        float B = start.pose.position.x - point.pose.position.x;
        float C = point.pose.position.x * start.pose.position.y - start.pose.position.x * start.pose.position.y;
        return std::abs(A * end.pose.position.x + B * end.pose.position.y + C) / std::sqrt(A * A + B * B);
    }

    void computePath(int start_x, int start_y, int goal_x, int goal_y) {
        if (isStraightLineClear(start_x, start_y, goal_x, goal_y)) {
            // Generate straight line path
            std::vector<geometry_msgs::msg::PoseStamped> path;
            geometry_msgs::msg::PoseStamped start_pose, goal_pose;
            start_pose.pose.position.x = start_x * map_.info.resolution;
            start_pose.pose.position.y = start_y * map_.info.resolution;
            start_pose.pose.position.z = 0.0;
            goal_pose.pose.position.x = goal_x * map_.info.resolution;
            goal_pose.pose.position.y = goal_y * map_.info.resolution;
            goal_pose.pose.position.z = 0.0;
            path.push_back(start_pose);
            path.push_back(goal_pose);
            publishPath(smoothPath(path, 0.5f)); // Smooth the path
            RCLCPP_INFO(this->get_logger(), "Straight line path found.");
            return;
        }

        auto heuristic = [this](int x1, int y1, int x2, int y2) -> float {
            // Use Euclidean distance for heuristic
            return std::sqrt(static_cast<float>((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
        };

        GridNode start_node{ start_x, start_y, 0.0f, heuristic(start_x, start_y, goal_x, goal_y), nullptr };
        std::unordered_map<std::pair<int, int>, std::shared_ptr<GridNode>, GridNodeHash> all_nodes;
        all_nodes[{start_x, start_y}] = std::make_shared<GridNode>(start_node);

        std::priority_queue<std::shared_ptr<GridNode>, std::vector<std::shared_ptr<GridNode>>, std::greater<>> open_list;
        open_list.push(all_nodes[{start_x, start_y}]);

        std::unordered_map<std::pair<int, int>, float, GridNodeHash> closed_list;

        bool path_found = false;
        std::vector<geometry_msgs::msg::PoseStamped> final_path;

        while (!open_list.empty()) {
            auto current = open_list.top();
            open_list.pop();

            if (current->x == goal_x && current->y == goal_y) {
                path_found = true;
                std::shared_ptr<GridNode> node = current;
                while (node) {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.pose.position.x = node->x * map_.info.resolution;
                    pose.pose.position.y = node->y * map_.info.resolution;
                    pose.pose.position.z = 0.0;
                    final_path.push_back(pose);
                    node = node->parent;
                }
                std::reverse(final_path.begin(), final_path.end());
                break;
            }

            closed_list[{current->x, current->y}] = current->g_cost;

            std::vector<std::pair<int, int>> neighbors = {
                {current->x + 1, current->y}, {current->x - 1, current->y},
                {current->x, current->y + 1}, {current->x, current->y - 1}
            };

            for (const auto& neighbor : neighbors) {
                int nx = neighbor.first;
                int ny = neighbor.second;

                if (nx < 0 || nx >= map_.info.width || ny < 0 || ny >= map_.info.height) {
                    continue; // Out of bounds
                }

                if (map_.data[ny * map_.info.width + nx] == 100) {
                    continue; // Occupied by obstacle
                }

                float obstacle_cost = getObstacleCost(nx, ny);
                float tentative_g_cost = current->g_cost + 2.0f + obstacle_cost; // Include obstacle cost
                auto neighbor_node = all_nodes.find({ nx, ny });
                if (neighbor_node == all_nodes.end()) {
                    auto new_node = std::make_shared<GridNode>();
                    new_node->x = nx;
                    new_node->y = ny;
                    new_node->g_cost = tentative_g_cost;
                    new_node->h_cost = heuristic(nx, ny, goal_x, goal_y);
                    new_node->parent = current;
                    all_nodes[{nx, ny}] = new_node;
                    open_list.push(new_node);
                }
                else if (tentative_g_cost < neighbor_node->second->g_cost) {
                    neighbor_node->second->g_cost = tentative_g_cost;
                    neighbor_node->second->parent = current;
                    open_list.push(neighbor_node->second);
                }
            }
        }

        if (path_found) {
            publishPath(smoothPath(final_path, 0.5f)); // Smooth the path
            RCLCPP_INFO(this->get_logger(), "Path computed successfully.");
        }
        else {
            RCLCPP_WARN(this->get_logger(), "No path found.");
        }
    }

    void publishPath(const std::vector<geometry_msgs::msg::PoseStamped>& path) {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
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
    bool initial_pose_set_ = false;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePlannerNode>());
    rclcpp::shutdown();
    return 0;
}

