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
// OpenMP
#include <omp.h>

#define _ENABLE_ATOMIC_ALIGNMENT_FIX
//******************************************************************************
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
    std::string STORAGE_MAP_TOPIC =
        param<std::string>("lrf_to_grid.topic_name.storage_map", "/storage_map");
    std::string LASER_TOPIC = param<std::string>("lrf_to_grid.topic_name.laser", "/scan");
    std::string CLOUD_TOPIC = param<std::string>("lrf_to_grid.topic_name.cloud", "/cloud");
    std::string MAP_CLOUD_TOPIC = param<std::string>("lrf_to_grid.topic_name.map_cloud", "/cloud");
    // frame
    MAP_FRAME = param<std::string>("lrf_to_grid.tf_frame.map_frame", "map");
    ROBOT_FRAME = param<std::string>("lrf_to_grid.tf_frame.robot_frame", "base_link");
    // setup
    SMAP_PUBLISH_RATE = param<double>("lrf_to_grid.storage_map.publish_rate", 1.0);
    MAP_QUE_SIZE = static_cast<size_t>(param<int>("lrf_to_grid.storage_map.que_size", 5));
    ADD_CELL = param<bool>("lrf_to_grid.add_cell", false);
    VOXELGRID_SIZE = param<double>("lrf_to_grid.filter.voxelgrid_size", 0.04);
    Z_MAX = param<double>("lrf_to_grid.filter.z_max", 1.0);
    Z_MIN = param<double>("lrf_to_grid.filter.z_min", 0.0);
    MAP_Z_MAX = param<double>("lrf_to_grid.filter.map_z_max", 1.0);
    MAP_Z_MIN = param<double>("lrf_to_grid.filter.map_z_min", 0.0);
    gmap_.info.resolution = param<double>("lrf_to_grid.grid_map_info.resolution", 1.0);
    WIDTH = param<double>("lrf_to_grid.grid_map_info.width", 50.0);
    HEIGHT = param<double>("lrf_to_grid.grid_map_info.height", 50.0);
    gmap_.info.width = std::round(WIDTH / gmap_.info.resolution);
    gmap_.info.height = std::round(HEIGHT / gmap_.info.resolution);
    gmap_.info.origin.position.x = -0.5 * WIDTH;
    gmap_.info.origin.position.y = -0.5 * HEIGHT;
    gmap_.resize(gmap_.info.width, gmap_.info.height);
    storage_map_.info = gmap_.info;
    storage_map_.resize(storage_map_.info.width, storage_map_.info.height);
    // init
    // publisher
    map_pub_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>(MAP_TOPIC, rclcpp::QoS(10).reliable());
    storage_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        STORAGE_MAP_TOPIC, rclcpp::QoS(10).reliable());
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
        CLOUD_TOPIC, rclcpp::QoS(10),
        [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
          get_cloud_frame_ = msg->header.frame_id;
          make_grid_map(*msg);
        });
    map_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        MAP_CLOUD_TOPIC, rclcpp::QoS(10),
        [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
          pcl::PointCloud<pcl::PointXYZ> in_cloud;
          pcl::fromROSMsg(*msg, in_cloud);
          // 座標変換 map -> cloud
          std::optional<pcl::PointCloud<pcl::PointXYZ>> trans_cloud = transform_pcl_pointcloud(tf_buffer_, msg->header.frame_id, get_cloud_frame_, in_cloud);
          if (!trans_cloud)
          {
            RCLCPP_ERROR(this->get_logger(), "transform error");
            return;
          }
          in_cloud = trans_cloud.value();
          // nan値除去
          std::vector<int> mapping;
          pcl::removeNaNFromPointCloud(in_cloud, in_cloud, mapping);
          in_cloud = voxelgrid_filter(in_cloud, VOXELGRID_SIZE, VOXELGRID_SIZE, VOXELGRID_SIZE);
          in_cloud = passthrough_filter("z", in_cloud, MAP_Z_MIN, MAP_Z_MAX);
          // 座標変換 claud -> map
          std::optional<pcl::PointCloud<pcl::PointXYZ>> map_trans_cloud = transform_pcl_pointcloud(tf_buffer_, get_cloud_frame_, MAP_FRAME, in_cloud);
          if (!map_trans_cloud)
          {
            RCLCPP_ERROR(this->get_logger(), "transform error");
            return;
          }
          map_cloud_ = map_trans_cloud.value();
        });
    timer_ = this->create_wall_timer(1s * SMAP_PUBLISH_RATE, [&]()
                                     {
#pragma omp parallel for
      for (auto & cell : storage_map_.data) cell = 0;
      pcl::PointCloud<pcl::PointXYZ> sum_cloud;
      for (const auto & map_cloud : cloud_que_) {
        sum_cloud += map_cloud;
      }
      sum_cloud += map_cloud_;
      debug_cloud_pub_->publish(make_ros_pointcloud2(make_header(MAP_FRAME, get_cloud_time_), sum_cloud));
      storage_map_ = make_gmap(storage_map_.info, sum_cloud);
      storage_map_pub_->publish(
        make_nav_gridmap(make_header(MAP_FRAME, get_cloud_time_), storage_map_)); });
  }

private:
  bool ADD_CELL;
  double SMAP_PUBLISH_RATE;
  double VOXELGRID_SIZE;
  double Z_MAX;
  double Z_MIN;
  double MAP_Z_MAX;
  double MAP_Z_MIN;
  double WIDTH;
  double HEIGHT;
  size_t MAP_QUE_SIZE;
  const std::array<Vector2d, 8> nb = {Vector2d{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {-1, 1}, {-1, -1}, {1, -1}};
  // param
  std::string MAP_FRAME;
  std::string ROBOT_FRAME;
  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;
  // timer
  rclcpp::TimerBase::SharedPtr timer_;
  // subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  // publisher
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr storage_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_pub_;
  // laser
  laser_geometry::LaserProjection projector_;
  // map
  GridMap gmap_;
  rclcpp::Time get_cloud_time_;
  std::vector<GridMap> gmap_que_;
  std::string get_cloud_frame_;
  pcl::PointCloud<pcl::PointXYZ> map_cloud_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> cloud_que_;
  GridMap storage_map_;

  GridMap make_gmap(map_info_t info, pcl::PointCloud<pcl::PointXYZ> cloud)
  {
    GridMap gmap;
    gmap.info = info;
    gmap.resize(gmap.info.width, gmap.info.height);
    // 初期化
#pragma omp parallel for
    for (auto &cell : gmap.data)
      cell = 0;
    // マップ作成
    // #pragma omp parallel for
    for (const auto point : cloud.points)
    {
      Vector2d p = gmap.get_grid_pos(
          conversion_vector2<pcl::PointXYZ, Vector2d>(point)); // グリッド上の位置取得
      if (gmap.is_contain(p))                                  // その点がgmap_.info.widthとgmap_.info.heightの範囲内かどうか
        gmap.set(p, GridMap::WALL_VALUE);
      if (ADD_CELL)
      {
        for (size_t i = 0; i < 8; i++)
        {
          Vector2d np = p + nb[i];
          if (gmap.is_contain(
                  np)) // その点がgmap_.info.widthとgmap_.info.heightの範囲内かどうか
            if (!gmap.is_wall(gmap_.at(np)))
            {
              // #pragma omp critical
              gmap.set(np, GridMap::WALL_VALUE);
            }
        }
      }
    }
    return gmap;
  }

  void make_grid_map(const sensor_msgs::msg::PointCloud2 &get_cloud)
  {
    if (!tf_buffer_.canTransform(
            get_cloud.header.frame_id, MAP_FRAME, get_cloud.header.stamp,
            tf2::durationFromSec(1.0)))
    { // 変換無いよ
      RCLCPP_WARN(
          this->get_logger(), "%s %s can not Transform", MAP_FRAME.c_str(),
          get_cloud.header.frame_id.c_str());
      return;
    }
    if (!tf_buffer_.canTransform(
            ROBOT_FRAME, MAP_FRAME, rclcpp::Time(0),
            tf2::durationFromSec(1.0)))
    { // 変換無いよ
      RCLCPP_WARN(
          this->get_logger(), "%s %s can not Transform", MAP_FRAME.c_str(), ROBOT_FRAME.c_str());
      return;
    }
    auto map_to_base_link = lookup_transform(tf_buffer_, ROBOT_FRAME, MAP_FRAME);
    if (map_to_base_link)
    {
      rclcpp::Time start_time = rclcpp::Clock().now();
      Pose3d base_link_pose = make_pose(map_to_base_link.value().transform);
      gmap_.info.origin.position.x = base_link_pose.position.x - 0.5 * WIDTH;
      gmap_.info.origin.position.y = base_link_pose.position.y - 0.5 * HEIGHT;
      gmap_.info.origin.position.z = base_link_pose.position.z;
      storage_map_.info.origin = gmap_.info.origin;
      // msg convert
      pcl::PointCloud<pcl::PointXYZ> in_cloud;
      pcl::fromROSMsg(get_cloud, in_cloud);
      if (in_cloud.empty())
      {
        RCLCPP_WARN(this->get_logger(), "cloud empty");
        return;
      }
      // nan値除去
      std::vector<int> mapping;
      pcl::removeNaNFromPointCloud(in_cloud, in_cloud, mapping);
      in_cloud = voxelgrid_filter(in_cloud, VOXELGRID_SIZE, VOXELGRID_SIZE, VOXELGRID_SIZE);
      in_cloud = passthrough_filter("z", in_cloud, Z_MIN, Z_MAX);
      // 座標変換
      std::optional<pcl::PointCloud<pcl::PointXYZ>> trans_cloud = transform_pcl_pointcloud(tf_buffer_, get_cloud.header.frame_id, MAP_FRAME, in_cloud);
      if (!trans_cloud)
      {
        RCLCPP_ERROR(this->get_logger(), "transform error");
        return;
      }
      // 点群保持
      cloud_que_.push_back(trans_cloud.value());
      if (cloud_que_.size() > MAP_QUE_SIZE)
        cloud_que_.erase(cloud_que_.begin()); // 古いもの削除(pop)
      // debug_cloud_pub_->publish(make_ros_pointcloud2(make_header(MAP_FRAME, get_cloud.header.stamp), trans_cloud.value()));
      gmap_ = make_gmap(gmap_.info, trans_cloud.value());
      map_pub_->publish(make_nav_gridmap(make_header(MAP_FRAME, get_cloud.header.stamp), gmap_));
      get_cloud_time_ = get_cloud.header.stamp;
      double proc_time =
          unit_cast<unit::time::s, unit::time::ms>((rclcpp::Clock().now() - start_time).seconds());
      RCLCPP_INFO(this->get_logger(), "proc_time:%f[ms]", proc_time);
    }
  }
};