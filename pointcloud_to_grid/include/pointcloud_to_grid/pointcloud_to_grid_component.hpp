#pragma once
#include <execution>
#include <iostream>
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
class PointCloudToGrid : public ExtensionNode
{
public:
  PointCloudToGrid(const rclcpp::NodeOptions &options) : PointCloudToGrid("", options) {}
  PointCloudToGrid(
      const std::string &name_space = "",
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : ExtensionNode("pointcloud_to_grid_node", name_space, options),
        tf_buffer_(this->get_clock()),
        listener_(tf_buffer_)
  {
    RCLCPP_INFO(this->get_logger(), "start pointcloud_to_grid_node");
    // get param
    std::string MAP_TOPIC = param<std::string>("pointcloud_to_grid.topic_name.map", "/map");
    std::string STORAGE_MAP_TOPIC = param<std::string>("pointcloud_to_grid.topic_name.storage_map", "/storage_map");
    std::string CLOUD_TOPIC = param<std::string>("pointcloud_to_grid.topic_name.cloud", "/cloud");
    // frame
    MAP_FRAME = param<std::string>("pointcloud_to_grid.tf_frame.map_frame", "map");
    // setup
    PERIOD = param<double>("pointcloud_to_grid.period", 0.001);
    MAP_QUE_SIZE = (size_t)param<int>("pointcloud_to_grid.que.size", 5);
    ADD_CELL = param<bool>("pointcloud_to_grid.add_cell", false);
    VOXELGRID_SIZE = param<double>("pointcloud_to_grid.filter.voxelgrid_size", 0.04);
    Z_MAX = param<double>("pointcloud_to_grid.filter.z_max", 1.0);
    Z_MIN = param<double>("pointcloud_to_grid.filter.z_min", 0.0);
    gmap_.info.resolution = param<double>("pointcloud_to_grid.grid_map_info.resolution", 1.0);
    double width = param<double>("pointcloud_to_grid.grid_map_info.width", 50.0);
    double height = param<double>("pointcloud_to_grid.grid_map_info.height", 50.0);
    gmap_.info.width = std::round(width / gmap_.info.resolution);
    gmap_.info.height = std::round(height / gmap_.info.resolution);
    gmap_.info.origin.position.x = -0.5 * width;
    gmap_.info.origin.position.y = -0.5 * height;
    gmap_.resize(gmap_.info.width, gmap_.info.height);
    storage_map_.info = gmap_.info;
    storage_map_.resize(storage_map_.info.width, storage_map_.info.height);
    // init
    // publisher
    map_pub_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>(MAP_TOPIC, rclcpp::QoS(10).reliable());
    storage_map_pub_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>(STORAGE_MAP_TOPIC, rclcpp::QoS(10).reliable());
    debug_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "pointcloud_to_grid/debug_points", rclcpp::QoS(10));
    // subscriber
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        CLOUD_TOPIC, rclcpp::QoS(10), [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
        rclcpp::Time start_time = rclcpp::Clock().now();
        sensor_msgs::msg::PointCloud2 get_cloud;
        get_cloud = *msg;
        std::optional<sensor_msgs::msg::PointCloud2> trans_cloud =
          transform_pointcloud2(tf_buffer_, MAP_FRAME, get_cloud);
        if (!trans_cloud) {
          RCLCPP_ERROR(this->get_logger(), "transform error");
          return;
        }
        // msg convert
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(trans_cloud.value(), cloud);
        if (cloud.empty()) {
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
        for (auto & cell : gmap_.data) cell = 0;
        // マップ作成
#pragma omp parallel for
        for (const auto point : cut_cloud.points) {
          Vector2d p = gmap_.get_grid_pos(
            conversion_vector2<pcl::PointXYZ, Vector2d>(point));  //グリッド上の位置取得
          if (gmap_.is_contain(p))  //その点がgmap_.info.widthとgmap_.info.heightの範囲内かどうか
            if (!gmap_.is_wall(gmap_.at(p))) gmap_.set(p, GridMap::WALL_VALUE);
          if (ADD_CELL) {
            for (size_t i = 0; i < 8; i++) {
              Vector2d np = p + nb[i];
              if (gmap_.is_contain(np))  //その点がgmap_.info.widthとgmap_.info.heightの範囲内かどうか
                if (!gmap_.is_wall(gmap_.at(np)))
                {
#pragma omp critical
                  gmap_.set(np, GridMap::WALL_VALUE);
                }
            }
          }
        }
        map_pub_->publish(make_nav_gridmap(make_header(MAP_FRAME, get_cloud.header.stamp),gmap_));
        //MAP保持
        gmap_que_.push_back(gmap_);//push
        if(gmap_que_.size() > MAP_QUE_SIZE) gmap_que_.erase(gmap_que_.begin());//古いもの削除(pop)
        get_cloud_time_ = get_cloud.header.stamp;
        double proc_time = unit_cast<unit::time::s,unit::time::ms>((rclcpp::Clock().now() - start_time).seconds());
        RCLCPP_INFO(this->get_logger(), "proc_time:%f[ms]", proc_time); });
    // timer
    timer_ = this->create_wall_timer(1s * PERIOD, [&]()
                                     {
#pragma omp parallel for
        for (auto & cell : storage_map_.data) cell = 0;
#pragma omp parallel for
        for(const auto &map : gmap_que_)
        {
          for (size_t i = 0;i < map.data.size();i++)
          {
            if (map.is_wall(map.data[i]))
            {
              if (!storage_map_.is_wall(storage_map_.data[i]))
              {
#pragma omp critical
                storage_map_.data[i] = GridMap::WALL_VALUE;
              }
            }
          }
        }
        storage_map_pub_->publish(make_nav_gridmap(make_header(MAP_FRAME, get_cloud_time_), storage_map_)); });
  }

private:
  bool ADD_CELL;
  double VOXELGRID_SIZE;
  double Z_MAX;
  double Z_MIN;
  double PERIOD;
  const std::array<Vector2d, 8> nb = {Vector2d{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};
  size_t MAP_QUE_SIZE;
  // param
  std::string MAP_FRAME;
  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;
  // timer
  rclcpp::TimerBase::SharedPtr timer_;
  // subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  // publisher
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr storage_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_pub_;
  // map
  rclcpp::Time get_cloud_time_;
  std::vector<GridMap> gmap_que_;
  GridMap gmap_;
  GridMap storage_map_;
};