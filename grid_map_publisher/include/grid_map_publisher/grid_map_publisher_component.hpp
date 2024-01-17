#pragma once
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "map_loader.hpp"

class GridMapPublisher : public rclcpp::Node
{
public:
  GridMapPublisher(const rclcpp::NodeOptions & options) : GridMapPublisher("", options) {}
  GridMapPublisher(
    const std::string & name_space = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("grid_map_publisher_node", name_space, options),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    map_loader_(*this)
  {
    tf_buffer_.setUsingDedicatedThread(true);

    declare_parameter<std::string>("map_yaml_filename", "field.yaml");
    std::string map_path = get_parameter("map_yaml_filename").as_string();
    if (map_loader_.load_file(map_path, "map", false)) {
      RCLCPP_INFO(this->get_logger(), "load map file");
    }

    if (map_loader_.exist_map_data()) {
      base_gmap_ = map_loader_.get_map();
      map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::SystemDefaultsQoS());
    }

    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(10ms, [&]() {
      if (!map_loader_.exist_map_data()) {
        return;
      }
      map_pub_->publish(*base_gmap_);
    });
  }

private:
  using Vec3 = Eigen::Vector3d;

private:
  MapLoader map_loader_;
  nav_msgs::msg::OccupancyGrid::SharedPtr base_gmap_;
  // TF
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::Buffer tf_buffer_;
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  // Publisher
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
};