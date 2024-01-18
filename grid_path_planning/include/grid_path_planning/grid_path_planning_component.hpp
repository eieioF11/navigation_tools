#pragma once
#include <execution>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
// Eigen
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
// ROS
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include "common_lib/ros2_utility/msg_util.hpp"
#include "common_lib/ros2_utility/tf_util.hpp"
#include "extension_node/extension_node.hpp"
// #include "common_lib/ros2_utility/marker_util.hpp"
// other
#include "common_lib/common_lib.hpp"

#define PLANNER_DEBUG_OUTPUT
#include "common_lib/planner/a_star.hpp"
#include "common_lib/planner/wave_propagation.hpp"

#define _ENABLE_ATOMIC_ALIGNMENT_FIX
//******************************************************************************
// for文の実行方法設定
//  #define LOOP_MODE std::execution::seq // 逐次実行
//  #define LOOP_MODE std::execution::par // 並列実行
#define LOOP_MODE std::execution::par_unseq  // 並列化・ベクトル化
// デバック関連設定
#define DEBUG_OUTPUT
//******************************************************************************
using namespace common_lib;
using namespace std::chrono_literals;
class GridPathPlanning : public ExtensionNode
{
public:
  GridPathPlanning(const rclcpp::NodeOptions & options) : GridPathPlanning("", options) {}
  GridPathPlanning(
    const std::string & name_space = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : ExtensionNode("grid_path_planning_node", name_space, options),
    broadcaster_(this),
    tf_buffer_(this->get_clock()),
    listener_(tf_buffer_)
  {
    RCLCPP_INFO(this->get_logger(), "start grid_path_planning_node");
    // get param
    std::string MAP_TOPIC = param<std::string>("grid_path_planning.topic_name.map", "/map");
    std::string TARGET_TOPIC =
      param<std::string>("grid_path_planning.topic_name.target", "/goal_pose");
    std::string GRIDPATH_TOPIC =
      param<std::string>("grid_path_planning.topic_name.grid_path", "grid_path_planning/grid_path");
    // frame
    MAP_FRAME = param<std::string>("grid_path_planning.tf_frame.map_frame", "map");
    ODOM_FRAME = param<std::string>("grid_path_planning.tf_frame.odom_frame", "odom");
    ROBOT_FRAME = param<std::string>("grid_path_planning.tf_frame.robot_frame", "base_link");
    // setup
    CONTROL_PERIOD = param<double>("grid_path_planning.control_period", 0.001);
    // init
    initialization_ = true;
    // planner_ = std::make_shared<WavePropagation>();
    planner_ = std::make_shared<AStar>();
    // publisher
    grid_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(GRIDPATH_TOPIC, rclcpp::QoS(10));
    // subscriber
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      MAP_TOPIC, rclcpp::QoS(10).reliable(),
      [&](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { map_msg_ = msg; });
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      TARGET_TOPIC, rclcpp::QoS(10),
      [&](geometry_msgs::msg::PoseStamped::SharedPtr msg) { target_pose_ = make_pose(msg->pose); });
    // timer
    main_timer_ = this->create_wall_timer(1s * CONTROL_PERIOD, [&]() {
      if (!tf_buffer_.canTransform(
            MAP_FRAME, ROBOT_FRAME, rclcpp::Time(0),
            tf2::durationFromSec(1.0))) {  // 変換無いよ
        RCLCPP_WARN(
          this->get_logger(), "%s %s can not Transform", MAP_FRAME.c_str(), ROBOT_FRAME.c_str());
        return;
      }
      auto map_to_base_link = lookup_transform(tf_buffer_, ROBOT_FRAME, MAP_FRAME);
      if (map_to_base_link) {
        base_link_pose_ = make_pose(map_to_base_link.value().transform);
        RCLCPP_INFO_CHANGE(0, this->get_logger(), "get base_link pose");
        if (map_msg_) {
          GridMap map = make_gridmap(*map_msg_);
          planner_->set_map(map);
          PathPointd start, end;
          RCLCPP_INFO_CHANGE(1, this->get_logger(), "get map");
          if (target_pose_) {
            std::cout << base_link_pose_ << std::endl;
            std::cout << map.info.resolution << std::endl;
            start.pose = base_link_pose_;
            end.pose = target_pose_.value();
            Pathd grid_path = planner_->pathplanning(start, end);
            std::cout << grid_path << std::endl;
            grid_path_pub_->publish(make_nav_path(make_header(MAP_FRAME, rclcpp::Clock().now()), grid_path));
            target_pose_ = std::nullopt;
          }
        }
      }
    });
  }

private:
  bool initialization_;
  // param
  std::string MAP_FRAME;
  std::string ROBOT_FRAME;
  std::string ODOM_FRAME;
  double CONTROL_PERIOD;
  double MIN_VEL;
  double MIN_ANGULAR;
  bool ODOM_TF;
  // マルチスレッド関連
  inline static std::mutex mtx;
  // tf
  tf2_ros::TransformBroadcaster broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;
  // timer
  rclcpp::TimerBase::SharedPtr main_timer_;
  // subscriber
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  // publisher
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr grid_path_pub_;
  // pose
  Pose3d odom_pose_;
  Pose3d imu_pose_;
  Pose3d base_link_pose_;
  std::optional<Pose3d> target_pose_;
  // vel
  Twistd odom_twist_;
  Twistd estimate_twist_;
  // map
  nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
  // planner
  std::shared_ptr<GridPathPlanner> planner_;

  Vector2d vector2_cast(const PathPointd & p) { return {p.pose.position.x, p.pose.position.y}; }
  PathPointd pathpoint_cast(const Vector2d & v)
  {
    PathPointd p;
    p.pose.position.x = v.x;
    p.pose.position.y = v.y;
    p.pose.position.z = 0.0;
    return p;
  }
};