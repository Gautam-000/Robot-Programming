#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

class RobotControllerNode : public rclcpp::Node {
public:
    RobotControllerNode() : Node("robot_controller_node"),
        current_index_(0),
        initial_pose_received_(false),
        goal_received_(false) {
        // Subscriber to the computed path
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/computed_path", 10, std::bind(&RobotControllerNode::pathCallback, this, std::placeholders::_1));

        // Subscriber to the initial pose
        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, std::bind(&RobotControllerNode::initialPoseCallback, this, std::placeholders::_1));

        // Subscriber to the final goal
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&RobotControllerNode::goalCallback, this, std::placeholders::_1));

        // Publisher for the robot's velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Publisher for the robot's current pose
        current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_pose", 10);

        // Timer to update the robot's position periodically
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&RobotControllerNode::updateRobotPosition, this));
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        path_ = *msg;
        current_index_ = 0;
        RCLCPP_INFO(this->get_logger(), "Path received with %zu waypoints.", path_.poses.size());
    }

    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        current_yaw_ = yaw;

        initial_pose_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Initial pose received: x=%.2f, y=%.2f, yaw=%.2f", current_x_, current_y_, current_yaw_);

        // Publish the initial pose
        publishCurrentPose();
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goal_pose_ = *msg;
        goal_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Goal pose received: x=%.2f, y=%.2f",
            goal_pose_.pose.position.x, goal_pose_.pose.position.y);
    }

    void updateRobotPosition() {
        // Check if both the initial pose and goal have been received
        if (!initial_pose_received_ || !goal_received_) {
            RCLCPP_INFO(this->get_logger(), "Waiting for initial pose and/or goal.");
            return;
        }

        if (current_index_ >= path_.poses.size()) {
            // No more waypoints to follow
            RCLCPP_INFO(this->get_logger(), "Reached final waypoint.");
            return;
        }

        // Get the current waypoint
        auto target_pose = path_.poses[current_index_];

        // Simple proportional controller to move towards the target waypoint
        double target_x = target_pose.pose.position.x;
        double target_y = target_pose.pose.position.y;

        // Calculate the angle to the target
        double angle_to_goal = atan2(target_y - current_y_, target_x - current_x_);
        double distance_to_goal = sqrt(pow(target_x - current_x_, 2) + pow(target_y - current_y_, 2));

        // Create a twist message to send velocity commands to the robot
        geometry_msgs::msg::Twist cmd_vel;
        if (distance_to_goal > 0.1) { // Move towards the target waypoint
            cmd_vel.linear.x = std::min(0.5, distance_to_goal); // Adjust speed based on distance
            cmd_vel.angular.z = 2.0 * (angle_to_goal - current_yaw_); // Proportional controller for orientation
            RCLCPP_INFO(this->get_logger(), "Moving towards waypoint: x=%.2f, y=%.2f", target_x, target_y);
        }
        else {
            // Reached the waypoint, stop and proceed to the next waypoint
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            current_index_++;
            RCLCPP_INFO(this->get_logger(), "Waypoint reached. Moving to next waypoint.");
        }

        // Check if the robot has reached the final goal pose
        double distance_to_goal_pose = sqrt(pow(goal_pose_.pose.position.x - current_x_, 2) +
            pow(goal_pose_.pose.position.y - current_y_, 2));

        if (distance_to_goal_pose < 0.1) {
            // Reached the goal pose
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Goal pose reached.");
        }

        // Publish the velocity command
        cmd_vel_pub_->publish(cmd_vel);

        // Update the current robot position (simplified for demonstration)
        current_x_ += cmd_vel.linear.x * cos(current_yaw_) * 0.1;
        current_y_ += cmd_vel.linear.x * sin(current_yaw_) * 0.1;
        current_yaw_ += cmd_vel.angular.z * 0.1;

        // Publish the updated current pose
        publishCurrentPose();
    }

    void publishCurrentPose() {
        geometry_msgs::msg::PoseStamped current_pose;
        current_pose.header.stamp = this->get_clock()->now();
        current_pose.header.frame_id = "map"; // Frame in which the pose is represented

        // Set the position
        current_pose.pose.position.x = current_x_;
        current_pose.pose.position.y = current_y_;
        current_pose.pose.position.z = 0.0;

        // Convert current_yaw_ to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, current_yaw_);
        current_pose.pose.orientation = tf2::toMsg(q);

        // Publish the current pose
        current_pose_pub_->publish(current_pose);
        RCLCPP_INFO(this->get_logger(), "Current pose published.");
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path path_;
    size_t current_index_;
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;
    geometry_msgs::msg::PoseStamped goal_pose_;
    bool initial_pose_received_;
    bool goal_received_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotControllerNode>());
    rclcpp::shutdown();
    return 0;
}
