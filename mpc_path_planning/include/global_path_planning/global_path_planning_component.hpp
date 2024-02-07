#pragma once
#include "mpc_path_planning/mpc_path_planning_component.hpp"
// opencv
#include <opencv2/opencv.hpp>

#include "cv_bridge/cv_bridge.h"
// OpenMP
#include <omp.h>

#define _ENABLE_ATOMIC_ALIGNMENT_FIX

using namespace common_lib;
using namespace std::chrono_literals;
class GlobalPathPlanning : public ExtensionNode
{
public:
  GlobalPathPlanning(const rclcpp::NodeOptions & options) : GlobalPathPlanning("", options) {}
  GlobalPathPlanning(
    const std::string & name_space = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : ExtensionNode("global_path_planning_node", name_space, options),
    tf_buffer_(this->get_clock()),
    listener_(tf_buffer_)
  {
    RCLCPP_INFO(this->get_logger(), "start global_path_planning_node");
    // get param
    std::string MAP_TOPIC = param<std::string>("global_path_planning.topic_name.map", "/map");
    std::string TARGET_TOPIC =
      param<std::string>("global_path_planning.topic_name.target", "/goal_pose");
    std::string GLOBALPATH_TOPIC = param<std::string>(
      "global_path_planning.topic_name.global_path", "global_path_planning/path");
    // frame
    MAP_FRAME = param<std::string>("global_path_planning.tf_frame.map_frame", "map");
    ROBOT_FRAME = param<std::string>("global_path_planning.tf_frame.robot_frame", "base_link");
    // setup
    CONTROL_PERIOD = param<double>("global_path_planning.planning_period", 0.001);
    THRESHOLD = param<int>("global_path_planning.dist_map.threshold", 20);
    ALPHA = param<double>("global_path_planning.dist_map.alpha", -0.2);
    BETA = param<double>("global_path_planning.dist_map.beta", -1.0);
    std::string PATH_PLANNER = param<std::string>("global_path_planning.path_planner", "a_star");
    // グリッドパスプランニング設定
    if (PATH_PLANNER.compare("wave_propagation") == 0)
      planner_ = std::make_shared<WavePropagation>();
    else if (PATH_PLANNER.compare("wave_propagation_dist_map") == 0)
      planner_ = std::make_shared<WavePropagation>(true);
    else if (PATH_PLANNER.compare("a_star") == 0)
      planner_ = std::make_shared<AStar>();
    else if (PATH_PLANNER.compare("extension_a_star") == 0)
      planner_ = std::make_shared<ExtensionAStar>();
    else if (PATH_PLANNER.compare("dijkstra") == 0)
      planner_ = std::make_shared<Dijkstra>();
    else if (PATH_PLANNER.compare("dijkstra_dist_map") == 0)
      planner_ = std::make_shared<Dijkstra>(true);
    else {
      RCLCPP_ERROR(this->get_logger(), "Nonexistent path planner algorithm");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "path planner: %s", PATH_PLANNER.c_str());
    // publisher
    global_path_pub_ =
      this->create_publisher<nav_msgs::msg::Path>(GLOBALPATH_TOPIC, rclcpp::QoS(10).reliable());
    gpp_perfomance_pub_ =
      this->create_publisher<std_msgs::msg::Float32>("global_path_planning/time", rclcpp::QoS(5));
    dist_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "global_path_planning/dist_map", rclcpp::QoS(10).reliable());
    // subscriber
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      MAP_TOPIC, rclcpp::QoS(10).reliable(),
      [&](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map_msg_ = msg;
        gen_distance_map_ = false;
      });
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      TARGET_TOPIC, rclcpp::QoS(10), [&](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "get target!");
        target_pose_ = make_pose(msg->pose);
      });
    end_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "mpc_path_planning/end", rclcpp::QoS(10).reliable(),
      [&](std_msgs::msg::Empty::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "end!");
        target_pose_ = std::nullopt;
      });
    // timer
    path_planning_timer_ = this->create_wall_timer(1s * CONTROL_PERIOD, [&]() {
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
          calc_distance_map();
          planner_->set_map(make_gridmap(dist_map_msg_));
          RCLCPP_INFO_CHANGE(1, this->get_logger(), "get map");
          if (target_pose_) {
            PathPointd start = make_pathpoint<double>(base_link_pose_);
            PathPointd end = make_pathpoint<double>(target_pose_.value());
            start_planning_timer_ = rclcpp::Clock().now();
            Pathd grid_path = planner_->path_planning(start, end);
            global_path_pub_->publish(
              make_nav_path(make_header(MAP_FRAME, rclcpp::Clock().now()), grid_path));
            double calc_time = (rclcpp::Clock().now() - start_planning_timer_).seconds();
            gpp_perfomance_pub_->publish(
              make_float32(unit_cast<unit::time::s, unit::time::ms>(calc_time)));
          }
        }
      }
    });
  }

private:
  // param
  std::string MAP_FRAME;
  std::string ROBOT_FRAME;
  double CONTROL_PERIOD;
  uint8_t THRESHOLD = 20;
  double ALPHA = -0.2;
  double BETA = -0.1;
  bool gen_distance_map_;
  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;
  // timer
  rclcpp::TimerBase::SharedPtr path_planning_timer_;
  rclcpp::Time start_create_distmap_time_;
  rclcpp::Time start_planning_timer_;
  // subscriber
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr end_sub_;
  // publisher
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr dist_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gpp_perfomance_pub_;
  // pose
  Pose3d base_link_pose_;
  std::optional<Pose3d> target_pose_;
  // map
  nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
  nav_msgs::msg::OccupancyGrid dist_map_msg_;
  // planner
  std::shared_ptr<GridPathPlanner> planner_;

  void calc_distance_map()
  {
    if (!map_msg_) return;
    if (gen_distance_map_) return;
#if defined(MAP_GEN_DEBUG_OUTPUT)
    std::cout << "calc distance map start!" << std::endl;
#endif
    start_create_distmap_time_ = rclcpp::Clock().now();
    dist_map_msg_.header = map_msg_->header;
    dist_map_msg_.info = map_msg_->info;
    dist_map_msg_.data.resize(dist_map_msg_.info.width * dist_map_msg_.info.height);
    Pose3d origin = make_pose(dist_map_msg_.info.origin);
    cv::Mat img = make_cv_mat(*map_msg_);
    cv::Mat filtering_img, convert_img;
    // 二値化
    // cv::threshold(img, img, 250, 255, cv::THRESH_BINARY);// unknown 255
    cv::threshold(img, img, 50, 255, cv::THRESH_BINARY);  // unknown 0
    // 距離場計算
    // cv::distanceTransform(img, filtering_im, CV_DIST_L1, 3);
    cv::distanceTransform(img, filtering_img, CV_DIST_L2, 5);  //ユークリッド距離
    cv::threshold(filtering_img, filtering_img, THRESHOLD, 255, cv::THRESH_TRUNC);
    filtering_img.convertTo(convert_img, CV_32FC1, 1.0);
    double min, max;
    cv::Point min_p, max_p;
    cv::minMaxLoc(convert_img, &min, &max, &min_p, &max_p);
#if defined(MAP_GEN_DEBUG_OUTPUT)
    std::cout << "min:" << min << " max:" << max << std::endl;
    std::cout << "alpha:" << ALPHA << " beta:" << BETA << std::endl;
#endif
#pragma omp parallel for
    for (size_t y = 0; y < dist_map_msg_.info.height; y++) {
      for (size_t x = 0; x < dist_map_msg_.info.width; x++) {
        // float pixel = transform_range<float>(
        //   max - convert_img.at<float>(y, x), min, max, 0.0, (float)GridMap::WALL_VALUE);
        float val = transform_range<float>(max - convert_img.at<float>(y, x), min, max, 0.0f, 1.0f);
        // float pixel = static_cast<float>(GridMap::WALL_VALUE)*(std::pow(10.0f,val-0.5f)-2.0f);
        // float pixel = static_cast<float>(GridMap::WALL_VALUE)*(std::exp(val-0.2f)-2.0f);
        float pixel = static_cast<float>(GridMap::WALL_VALUE) *
                      (std::exp(val + static_cast<float>(ALPHA)) + static_cast<float>(BETA));
        // float pixel = static_cast<float>(GridMap::WALL_VALUE)*val;
        if (pixel >= (float)GridMap::WALL_VALUE) pixel = (float)GridMap::WALL_VALUE;
        if (pixel < 0.0) pixel = 0.0;
        dist_map_msg_.data[dist_map_msg_.info.width * (dist_map_msg_.info.height - y - 1) + x] =
          static_cast<int8_t>(pixel);
      }
    }
    dist_map_pub_->publish(dist_map_msg_);
#if defined(MAP_GEN_DEBUG_OUTPUT)
    double calc_time = (rclcpp::Clock().now() - start_create_distmap_time_).seconds();
    std::cout << "calc_time:" << calc_time << std::endl;
    std::cout << "calc distance map end!" << std::endl;
#endif
    gen_distance_map_ = true;
  }
};