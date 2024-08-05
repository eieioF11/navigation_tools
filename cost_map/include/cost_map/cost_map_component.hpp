#pragma once
#include <execution>
#include <iostream>
#include <laser_geometry/laser_geometry.hpp>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
// PCL
#include "common_lib/common_lib.hpp"
#include "common_lib/pcl_utility/pcl_util.hpp"
#include "common_lib/ros2_utility/msg_util.hpp"
#include "common_lib/ros2_utility/ros_pcl_util.hpp"
#include "common_lib/ros2_utility/tf_util.hpp"
#include "extension_node/extension_node.hpp"
#include "common_lib/ros2_utility/ros_opencv_util.hpp"
// OpenMP
#include <omp.h>
// opencv
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"

#define _ENABLE_ATOMIC_ALIGNMENT_FIX
//******************************************************************************
// デバック関連設定
#define DEBUG_OUTPUT
//******************************************************************************
using namespace common_lib;
using namespace std::chrono_literals;
class CostMap : public ExtensionNode
{
public:
  CostMap(const rclcpp::NodeOptions &options) : CostMap("", options) {}
  CostMap(
      const std::string &name_space = "",
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : ExtensionNode("cost_map_node", name_space, options)
  {
    RCLCPP_INFO(this->get_logger(), "start cost_map_node");
    // get param
    std::string MAP_TOPIC = param<std::string>("cost_map.topic_name.map", "/map");
    std::string COST_MAP_TOPIC = param<std::string>("cost_map.topic_name.cost_map", "/cost_map");
    // setup
    double PUBLISH_RATE = param<double>("cost_map.publish_rate", 1.0);
    // setup
    THRESHOLD = param<int>("cost_map.threshold", 20);
    ALPHA = param<double>("cost_map.alpha", -0.2);
    BETA = param<double>("cost_map.beta", -1.0);
    // init
    // publisher
    map_pub_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>(COST_MAP_TOPIC, rclcpp::QoS(10).reliable());
    // subscriber
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        MAP_TOPIC, rclcpp::QoS(10), [&](const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
        { cost_map_msg_ = calc_cost_map(*msg); });
    timer_ = this->create_wall_timer(1s * PUBLISH_RATE, [&]()
                                     { map_pub_->publish(cost_map_msg_); });
  }

private:
  static constexpr int8_t WALL_VALUE = 100;
  uint8_t THRESHOLD = 20;
  double ALPHA = -0.2;
  double BETA = -0.1;
  // timer
  rclcpp::TimerBase::SharedPtr timer_;
  // subscriber
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  // publisher
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  // map
  nav_msgs::msg::OccupancyGrid cost_map_msg_;

  nav_msgs::msg::OccupancyGrid calc_cost_map(const nav_msgs::msg::OccupancyGrid &map)
  {
    nav_msgs::msg::OccupancyGrid cost_map;
    std::cout << "calc cost  map start!" << std::endl;
    auto start_time = rclcpp::Clock().now();
    cost_map.header = map.header;
    cost_map.info = map.info;
    cost_map.data.resize(cost_map.info.width * cost_map.info.height);
    Pose3d origin = make_pose(cost_map.info.origin);
    cv::Mat img = make_cv_mat(map);
    cv::Mat filtering_img, convert_img;
    // 二値化
    cv::threshold(img, img, 50, 255, cv::THRESH_BINARY); // unknown 0
    // 距離場計算
    cv::distanceTransform(img, filtering_img, CV_DIST_L2, 5); // ユークリッド距離
    cv::threshold(filtering_img, filtering_img, THRESHOLD, 255, cv::THRESH_TRUNC);
    filtering_img.convertTo(convert_img, CV_32FC1, 1.0);
    double min, max;
    cv::Point min_p, max_p;
    cv::minMaxLoc(convert_img, &min, &max, &min_p, &max_p);
    std::cout << "min:" << min << " max:" << max << std::endl;
    std::cout << "alpha:" << ALPHA << " beta:" << BETA << std::endl;
#pragma omp parallel for
    for (size_t y = 0; y < cost_map.info.height; y++)
    {
      for (size_t x = 0; x < cost_map.info.width; x++)
      {
        float val = transform_range<float>(max - convert_img.at<float>(y, x), min, max, 0.0f, 1.0f);
        float pixel = static_cast<float>(CostMap::WALL_VALUE) *
                      (std::exp(val + static_cast<float>(ALPHA)) + static_cast<float>(BETA));
        if (pixel >= (float)CostMap::WALL_VALUE)
          pixel = (float)CostMap::WALL_VALUE;
        if (pixel < 0.0)
          pixel = 0.0;
        cost_map.data[cost_map.info.width * (cost_map.info.height - y - 1) + x] =
            static_cast<int8_t>(pixel);
      }
    }
    double calc_time = (rclcpp::Clock().now() - start_time).seconds();
    std::cout << "calc_time:" << calc_time << std::endl;
    std::cout << "calc distance map end!" << std::endl;
    return cost_map;
  }
};