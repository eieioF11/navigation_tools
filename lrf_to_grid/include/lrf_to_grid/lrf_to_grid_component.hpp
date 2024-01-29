#pragma once
#include <execution>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <laser_geometry/laser_geometry.hpp>
// PCL
#include "common_lib/common_lib.hpp"
#include "common_lib/pcl_utility/pcl_util.hpp"
#include "common_lib/ros2_utility/msg_util.hpp"
#include "common_lib/ros2_utility/ros_pcl_util.hpp"
#include "extension_node/extension_node.hpp"
// OpenMP
#include <omp.h>

#define _ENABLE_ATOMIC_ALIGNMENT_FIX
//******************************************************************************
// for文の実行方法設定
//  #define LOOP_MODE std::execution::seq // 逐次実行
//  #define LOOP_MODE std::execution::par // 並列実行
#define LOOP_MODE std::execution::par_unseq // 並列化・ベクトル化
// デバック関連設定
#define DEBUG_OUTPUT
//******************************************************************************
using namespace common_lib;
using namespace std::chrono_literals;
class LRFToGrid : public ExtensionNode
{
public:
  LRFToGrid(const rclcpp::NodeOptions &options) : LRFToGrid("", options) {}
  LRFToGrid(
      const std::string &name_space = "",
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : ExtensionNode("lrf_to_grid_node", name_space, options),
        tf_buffer_(this->get_clock()),
        listener_(tf_buffer_)
  {
    RCLCPP_INFO(this->get_logger(), "start lrf_to_grid_node");
    // get param
    std::string MAP_TOPIC = param<std::string>("lrf_to_grid.topic_name.map", "/map");
    std::string LASER_TOPIC = param<std::string>("lrf_to_grid.topic_name.laser", "/scan");
    std::string CLOUD_TOPIC = param<std::string>("lrf_to_grid.topic_name.cloud", "/cloud");
    // frame
    MAP_FRAME = param<std::string>("lrf_to_grid.tf_frame.map_frame", "map");
    // setup
    ADD_CELL = param<bool>("lrf_to_grid.add_cell", false);
    VOXELGRID_SIZE = param<double>("lrf_to_grid.filter.voxelgrid_size", 0.04);
    Z_MAX = param<double>("lrf_to_grid.filter.z_max", 1.0);
    Z_MIN = param<double>("lrf_to_grid.filter.z_min", 0.0);
    gmap_.info.resolution = param<double>("lrf_to_grid.grid_map_info.resolution", 1.0);
    double width = param<double>("lrf_to_grid.grid_map_info.width", 50.0);
    double height = param<double>("lrf_to_grid.grid_map_info.height", 50.0);
    gmap_.info.width = std::round(width / gmap_.info.resolution);
    gmap_.info.height = std::round(height / gmap_.info.resolution);
    gmap_.info.origin.position.x = -0.5 * width;
    gmap_.info.origin.position.y = -0.5 * height;
    gmap_.resize(gmap_.info.width, gmap_.info.height);
    // init
    // publisher
    map_pub_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>(MAP_TOPIC, rclcpp::QoS(10).reliable());
    debug_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "lrf_to_grid/debug_points", rclcpp::QoS(10));
    // subscriber
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        LASER_TOPIC, rclcpp::QoS(10), [&](const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
          sensor_msgs::msg::PointCloud2 get_cloud;
          projector_.projectLaser(*msg, get_cloud);
          get_cloud.header = msg->header;
          make_grid_map(get_cloud); });
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        CLOUD_TOPIC, rclcpp::QoS(10), [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        { make_grid_map(*msg); });
  }

private:
  bool ADD_CELL;
  double VOXELGRID_SIZE;
  double Z_MAX;
  double Z_MIN;
  const std::array<Vector2d, 8> nb = {Vector2d{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};
  // param
  std::string MAP_FRAME;
  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;
  // subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  // publisher
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_pub_;
  // laser
  laser_geometry::LaserProjection projector_;
  // map
  GridMap gmap_;

  void make_grid_map(const sensor_msgs::msg::PointCloud2 &get_cloud)
  {
    if (!tf_buffer_.canTransform(
            get_cloud.header.frame_id, MAP_FRAME, get_cloud.header.stamp,
            tf2::durationFromSec(1.0)))
    { // 変換無いよ
      RCLCPP_WARN(
          this->get_logger(), "%s %s can not Transform", MAP_FRAME.c_str(), get_cloud.header.frame_id.c_str());
      return;
    }
    rclcpp::Time start_time = rclcpp::Clock().now();
    std::optional<sensor_msgs::msg::PointCloud2> trans_cloud =
        transform_pointcloud2(tf_buffer_, MAP_FRAME, get_cloud);
    if (!trans_cloud)
    {
      RCLCPP_ERROR(this->get_logger(), "transform error");
      return;
    }
    // msg convert
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(trans_cloud.value(), cloud);
    if (cloud.empty())
    {
      RCLCPP_WARN(this->get_logger(), "cloud empty");
      return;
    }
    // nan値除去
    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(cloud, cloud, mapping);
    cloud = voxelgrid_filter(cloud, VOXELGRID_SIZE, VOXELGRID_SIZE, VOXELGRID_SIZE);
    pcl::PointCloud<pcl::PointXYZ> cut_cloud = passthrough_filter("z", cloud, Z_MIN, Z_MAX);
    debug_cloud_pub_->publish(
        make_ros_pointcloud2(make_header(MAP_FRAME, get_cloud.header.stamp), cut_cloud));
    // 初期化
#pragma omp parallel for
    for (auto &cell : gmap_.data)
      cell = 0;
      // マップ作成
#pragma omp parallel for
    for (const auto point : cut_cloud.points)
    {
      Vector2d p = gmap_.get_grid_pos(
          conversion_vector2<pcl::PointXYZ, Vector2d>(point)); // グリッド上の位置取得
      if (gmap_.is_contain(p))                                 // その点がgmap_.info.widthとgmap_.info.heightの範囲内かどうか
        if (!gmap_.is_wall(gmap_.at(p)))
          gmap_.set(p, GridMap::WALL_VALUE);
      if (ADD_CELL)
      {
        for (size_t i = 0; i < 8; i++)
        {
          Vector2d np = p + nb[i];
          if (gmap_.is_contain(np)) // その点がgmap_.info.widthとgmap_.info.heightの範囲内かどうか
            if (!gmap_.is_wall(gmap_.at(np)))
            {
#pragma omp critical
              gmap_.set(np, GridMap::WALL_VALUE);
            }
        }
      }
    }
    map_pub_->publish(make_nav_gridmap(make_header(MAP_FRAME, get_cloud.header.stamp), gmap_));
    double proc_time = unit_cast<unit::time::s, unit::time::ms>((rclcpp::Clock().now() - start_time).seconds());
    RCLCPP_INFO(this->get_logger(), "proc_time:%f[ms]", proc_time);
  }
};