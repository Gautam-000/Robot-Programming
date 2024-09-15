#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <opencv2/opencv.hpp>

class GridMapPublisher : public rclcpp::Node {
public:
    GridMapPublisher() : Node("grid_map_publisher") {
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/grid_map", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Load and convert the image to occupancy grid
        if (loadMapFromImage("/home/baba/ros2_humble_ws/src/simple_planner/maps/cappero.png")) {
            RCLCPP_INFO(this->get_logger(), "Map loaded successfully.");
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Failed to load map from image.");
            return;
        }

        // Create a timer to periodically publish the map and the transform
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&GridMapPublisher::publishMapAndTF, this));

        RCLCPP_INFO(this->get_logger(), "GridMapPublisher node initialized.");
    }

private:
    bool loadMapFromImage(const std::string& file_path) {
        cv::Mat image = cv::imread(file_path, cv::IMREAD_GRAYSCALE);
        if (image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open image file: %s", file_path.c_str());
            return false;
        }

        map_.header.frame_id = "map";
        map_.info.resolution = 0.02; // Example resolution in meters per cell
        map_.info.width = image.cols;
        map_.info.height = image.rows;
        map_.info.origin.position.x = 0.0;
        map_.info.origin.position.y = 0.0;
        map_.info.origin.position.z = 0.0;
        map_.info.origin.orientation.w = 1.0;

        map_.data.resize(map_.info.width * map_.info.height);

        for (int y = 0; y < image.rows; ++y) {
            for (int x = 0; x < image.cols; ++x) {
                uint8_t pixel = image.at<uint8_t>(y, x);
                if (pixel > 210) {
                    map_.data[y * map_.info.width + x] = 0;  // Free space (white)
                }
                else {
                    map_.data[y * map_.info.width + x] = 100;  // Occupied space (everything else)
                }
            }
        }

        return true;
    }

    void publishMapAndTF() {
        // Update the header timestamp
        map_.header.stamp = this->get_clock()->now();

        // Publish the map
        publisher_->publish(map_);

        // Publish the transform from 'world' to 'map' frame
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = "world"; // Parent frame
        transformStamped.child_frame_id = "map";    // Child frame
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(transformStamped);

        RCLCPP_INFO(this->get_logger(), "Published grid map from image to /grid_map topic and TF from world to map.");
    }

    nav_msgs::msg::OccupancyGrid map_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GridMapPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
