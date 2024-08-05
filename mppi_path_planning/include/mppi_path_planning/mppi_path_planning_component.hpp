#pragma once
#include <execution>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <queue>
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
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
// common_lib
#define USE_OMP
#include "common_lib/common_lib.hpp"
// common_lib utility
#include "common_lib/ros2_utility/extension_msgs_util.hpp"
#include "common_lib/ros2_utility/jsk_msgs_util.hpp"
#include "common_lib/ros2_utility/marker_util.hpp"
#include "common_lib/ros2_utility/msg_util.hpp"
#include "common_lib/ros2_utility/ros_opencv_util.hpp"
#include "common_lib/ros2_utility/tf_util.hpp"
#include "extension_node/extension_node.hpp"

#include "mppi/mppi.hpp"

#define _ENABLE_ATOMIC_ALIGNMENT_FIX
//******************************************************************************
// モデル設定
// #define HOLONOMIC
// デバック関連設定
// #define PLANNING_DEBUG_OUTPUT
// #define MAP_GEN_DEBUG_OUTPUT
// #define CONTROL_DEBUG_OUTPUT
// #define USE_IMU_DEBUG
// #define OBSTACLE_DETECT_DEBUG_OUTPUT
//******************************************************************************
using namespace common_lib;
using namespace std::chrono_literals;
class MPPIPathPlanning : public ExtensionNode
{
public:
  MPPIPathPlanning(const rclcpp::NodeOptions &options) : MPPIPathPlanning("", options) {}
  MPPIPathPlanning(
      const std::string &name_space = "",
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : ExtensionNode("mppi_path_planning_node", name_space, options),
        tf_buffer_(get_clock()),
        listener_(tf_buffer_)
  {
    RCLCPP_INFO(get_logger(), "start mppi_path_planning_node");
    THREAD_NUM = omp_get_max_threads();
    x_tar_ = std::nullopt;
    // frame
    MAP_FRAME = param<std::string>("mppi_path_planning.tf_frame.map_frame", "map");
    ROBOT_FRAME = param<std::string>("mppi_path_planning.tf_frame.robot_frame", "base_link");
    // topic name
    std::string MAP_TOPIC =
        param<std::string>("mppi_path_planning.topic_name.map", "/lrf/storage_map");
    std::string TARGET_TOPIC =
        param<std::string>("mppi_path_planning.topic_name.target", "/goal_pose");
    std::string ODOM_TOPIC = param<std::string>("mppi_path_planning.topic_name.odom", "/odom");
    std::string CMD_VEL_TOPIC = param<std::string>("mppi_path_planning.topic_name.cmd_vel", "/auto/cmd_vel");
    // param
    PLANNING_PERIOD = param<double>("mppi_path_planning.planning_period", 0.1);
    // 収束判定
    GOAL_POS_RANGE = param<double>("mppi_path_planning.goal.pos_range", 0.01);
    GOAL_ANGLE_RANGE = unit_cast<unit::angle::rad>(param<double>("mppi_path_planning.goal.angle_range", 0.1));
    GOAL_MIN_VEL_RANGE = param<double>("mppi_path_planning.goal.min_vel_range", 0.001);
    GOAL_MIN_ANGULAR_RANGE = param<double>("mppi_path_planning.goal.min_angular_range", 0.001);
    // mppi
    MPPI::param_t mppi_param;
    mppi_param.T = param<int>("mppi_path_planning.mppi.T", 40);
    mppi_param.K = param<int>("mppi_path_planning.mppi.K", 500);
    mppi_param.J = param<double>("mppi_path_planning.mppi.J", 0.1);
    mppi_param.dt = param<double>("mppi_path_planning.mppi.dt", 0.01);
    mppi_param.lambda = param<double>("mppi_path_planning.mppi.lambda", 1.0);
    mppi_param.alpha = param<double>("mppi_path_planning.mppi.alpha", 0.2);
    std::vector<double> sigma_vec = param<std::vector<double>>(
        "mppi_path_planning.mppi.sigma", std::vector<double>{0.5, 0.0001, 2.5});
    std::vector<double> state_weight = param<std::vector<double>>(
        "mppi_path_planning.mppi.weight.state", std::vector<double>{10.0, 0, 1.0, 800.0, 800.0, 0.0});
    std::vector<double> control_weight = param<std::vector<double>>(
        "mppi_path_planning.mppi.weight.control", std::vector<double>{2.0, 0.0, 5.0});
    std::vector<double> terminal_weight = param<std::vector<double>>(
        "mppi_path_planning.mppi.weight.terminal", std::vector<double>{0.0, 0.0, 0.0, 1200.0, 1200.0, 1.0});
    mppi_param.window_size = param<double>("mppi_path_planning.mppi.window_size", 70.0);
    mppi_param.obstacle_cost = param<double>("mppi_path_planning.mppi.obstacle_cost", 2000.0);
    std::vector<double> min_vel = param<std::vector<double>>(
        "mppi_path_planning.min_vel", std::vector<double>{-0.3, 0.0, -1.0});
    std::vector<double> max_vel = param<std::vector<double>>(
        "mppi_path_planning.max_vel", std::vector<double>{0.3, 0.0, 1.0});
    Eigen::Matrix<double, 3, 3> sigma;
    sigma << sigma_vec[0], 0.0, 0.0,
        0.0, sigma_vec[1], 0.0,
        0.0, 0.0, sigma_vec[2];
    Eigen::Matrix<double, 6, 6> Q;
    Q << state_weight[0], 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, state_weight[1], 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, state_weight[2], 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, state_weight[3], 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, state_weight[4], 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, state_weight[5];
    Eigen::Matrix<double, 6, 6> Q_T;
    Q_T << terminal_weight[0], 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, terminal_weight[1], 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, terminal_weight[2], 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, terminal_weight[3], 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, terminal_weight[4], 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, terminal_weight[5];
    Eigen::Matrix<double, 3, 3> R;
    R << control_weight[0], 0.0, 0.0,
        0.0, control_weight[1], 0.0,
        0.0, 0.0, control_weight[2];
    mppi_param.sigma = sigma;
    mppi_param.Q = Q;
    mppi_param.R = R;
    mppi_param.Q_T = Q_T;
    mppi_ = std::make_shared<MPPI::MPPIPathPlanner>(mppi_param, std::bind(&MPPIPathPlanning::f, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    mppi_->set_velocity_limit({min_vel[0], min_vel[1], min_vel[2]}, {max_vel[0], max_vel[1], max_vel[2]});
    // publisher
    init_data_logger({"vx", "vy", "w", "u_vx", "u_vy", "u_w", "x_t", "y_t", "theta_t", "x_tar", "y_tar", "theta_tar"});
    cmd_vel_pub_ =
        create_publisher<geometry_msgs::msg::Twist>(CMD_VEL_TOPIC, rclcpp::QoS(10));
    opti_path_pub_ =
        create_publisher<nav_msgs::msg::Path>("mppi/opti_path", rclcpp::QoS(10).reliable());
    sample_path_marker_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("mppi/sample_path", rclcpp::QoS(10));
    mppi_calc_time_pub_ = create_publisher<std_msgs::msg::Float32>("mppi/calc_time", rclcpp::QoS(10));
    // subscriber
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        MAP_TOPIC, rclcpp::QoS(10).reliable(),
        [&](const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
        {
          mppi_->set_map(make_map(*msg));
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        ODOM_TOPIC, rclcpp::QoS(10),
        [&](nav_msgs::msg::Odometry::SharedPtr msg)
        { odom_vel_ = msg->twist.twist; });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        TARGET_TOPIC, rclcpp::QoS(10), [&](geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
          x_tar_ = pose_to_vec6_t(msg->pose);
          if(x_tar_)
          {
            log_.set("x_tar", x_tar_.value()(3));
            log_.set("y_tar", x_tar_.value()(4));
            log_.set("theta_tar", x_tar_.value()(5));
          } });
    // service
    reset_srv_ = create_service<std_srvs::srv::SetBool>(
        "planner/reset", [&](
                             const std_srvs::srv::SetBool::Request::SharedPtr req,
                             const std_srvs::srv::SetBool::Response::SharedPtr res)
        {
        res->success = true;
        res->message = req->data ? "planner reset!" : " ";
        if (req->data) {
          x_tar_ = std::nullopt;
          cmd_vel_pub_->publish(stop());
          RCLCPP_INFO(get_logger(), "planner reset!");
        } });
    // timer
    timer_ = this->create_wall_timer(1s * PLANNING_PERIOD, [&]()
                                     {
      if (!tf_buffer_.canTransform(
            MAP_FRAME, ROBOT_FRAME, rclcpp::Time(0),
            tf2::durationFromSec(1.0))) {  // 変換無いよ
        RCLCPP_WARN(
          get_logger(), "%s %s can not Transform", MAP_FRAME.c_str(), ROBOT_FRAME.c_str());
        return;
      }
      auto map_to_base_link = lookup_transform(tf_buffer_, ROBOT_FRAME, MAP_FRAME);
      if (map_to_base_link) {
        auto transform = map_to_base_link.value().transform;
        x_t_(0) = odom_vel_.linear.x;
        x_t_(1) = odom_vel_.linear.y;
        x_t_(2) = odom_vel_.angular.z;
        x_t_(3) = transform.translation.x;
        x_t_(4) = transform.translation.y;
        x_t_(5) = tf2::getYaw(transform.rotation);
        // log設定
        log_.set("vx", x_t_(0));
        log_.set("vy", x_t_(1));
        log_.set("w", x_t_(2));
        log_.set("x_t", x_t_(3));
        log_.set("y_t", x_t_(4));
        log_.set("theta_t", x_t_(5));
      }
      if (!x_tar_) {
        cmd_vel_pub_->publish(stop());
        return;
      }
      std::cout << "MAX threads NUM:" << THREAD_NUM << std::endl;
      std::cout << "x_t:" << x_t_.transpose() << std::endl;
      std::cout << "x_tar:" << x_tar_.value().transpose() << std::endl;
      // mppi計算
      const std::vector<MPPI::vec3_t> &u = mppi_->path_planning(x_t_, x_tar_.value());
      const std::vector<MPPI::vec6_t> &opt_path = mppi_->get_opt_path();
      const std::vector<std::vector<MPPI::vec6_t>> sample_path = mppi_->get_sample_path();
      MPPI::vec3_t v_t;
      v_t = u[0];
      // log設定
      log_.set("u_vx", v_t(0));
      log_.set("u_vy", v_t(1));
      log_.set("u_w", v_t(2));
      std::cout << "v_t:" << v_t.transpose() << std::endl;
      std::cout << "diff_x:" << (x_tar_.value()-x_t_).tail(3).head(2).transpose() << std::endl;
      double target_dist = (x_tar_.value()-x_t_).tail(3).head(2).norm();
      double target_diff_angle = x_tar_.value().tail(1).norm()-x_t_.tail(1).norm();
      std::cout << "target_dist:" << target_dist << " target_diff_angle:" << target_diff_angle << std::endl;
      target_diff_angle = 0.0;
      // ゴール判定
      if (target_dist < GOAL_POS_RANGE) {
        if (target_diff_angle < GOAL_ANGLE_RANGE) {
          RCLCPP_INFO(get_logger(), "goal !");
          x_tar_ = std::nullopt;
          cmd_vel_pub_->publish(stop());
          // if (
          //   approx_zero(vel, GOAL_MIN_VEL_RANGE) && approx_zero(angular, GOAL_MIN_ANGULAR_RANGE)) {
          //   RCLCPP_INFO(get_logger(), "goal !");
          //   opti_twists_ = std::nullopt;
          //   opti_path_ = std::nullopt;
          //   cmd_vel_pub_->publish(stop());
          //   return true;
          // }
        }
      }
      geometry_msgs::msg::Twist twist = make_twist(v_t);
      cmd_vel_pub_->publish(twist); 
      mppi_calc_time_pub_->publish(make_float32(mppi_->get_calc_time()));
      nav_msgs::msg::Path opti_path_msg = make_nav_path(make_header(MAP_FRAME, rclcpp::Clock().now()), opt_path);
      opti_path_pub_->publish(opti_path_msg);
      visualization_msgs::msg::MarkerArray samples_marker =  make_samples_marker(make_header(MAP_FRAME, rclcpp::Clock().now()),sample_path);
      sample_path_marker_pub_->publish(samples_marker);
      // log設定
      log_.publish(); });
  }

private:
  int THREAD_NUM;
  // param
  std::string MAP_FRAME;
  std::string ROBOT_FRAME;
  double PLANNING_PERIOD;
  double GOAL_POS_RANGE;
  double GOAL_ANGLE_RANGE;
  double GOAL_MIN_VEL_RANGE;
  double GOAL_MIN_ANGULAR_RANGE;
  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;
  // timer
  rclcpp::TimerBase::SharedPtr timer_;
  // subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  // publisher
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr opti_path_pub_;
  // rclcpp::Publisher<extension_msgs::msg::TwistMultiArray>::SharedPtr opti_twists_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mppi_calc_time_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sample_path_marker_pub_;
  // service
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr reset_srv_;
  // vel
  geometry_msgs::msg::Twist odom_vel_;
  // mppi
  std::optional<MPPI::vec6_t> x_tar_;
  MPPI::vec6_t x_t_;
  std::shared_ptr<MPPI::MPPIPathPlanner> mppi_;
#ifdef HOLONOMIC
  MPPI::vec6_t f(MPPI::vec6_t x_t, MPPI::vec3_t v_t, double dt)
  {
    MPPI::vec6_t x_next;
    Eigen::Matrix<double, 6, 6> A;
    A << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix<double, 6, 3> B;
    B << 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0;
    x_next = A * x_t + dt * B * v_t;
    return x_next;
  }
#else
  MPPI::vec6_t f(MPPI::vec6_t x_t, MPPI::vec3_t v_t, double dt)
  {
    MPPI::vec6_t x_next;
    Eigen::Matrix<double, 6, 6> A;
    A << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix<double, 6, 3> B;
    B << 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        std::cos(x_t(5)), 0.0, 0.0,
        std::sin(x_t(5)), 0.0, 0.0,
        0.0, 0.0, 1.0;
    x_next = A * x_t + dt * B * v_t;
    return x_next;
  }
#endif
  // utility
  MPPI::vec6_t pose_to_vec6_t(const geometry_msgs::msg::Pose &pose, double vx = 0.0, double vy = 0.0, double w = 0.0)
  {
    MPPI::vec6_t v;
    v(0) = vx;
    v(1) = vy;
    v(2) = w;
    v(3) = pose.position.x;
    v(4) = pose.position.y;
    v(5) = tf2::getYaw(pose.orientation);
    return v;
  }
  MPPI::GridMap make_map(const nav_msgs::msg::OccupancyGrid &gmap_msg)
  {
    MPPI::GridMap gmap;
    gmap.info.height = gmap_msg.info.height;
    gmap.info.width = gmap_msg.info.width;
    gmap.info.resolution = gmap_msg.info.resolution;
    gmap.info.origin_x = gmap_msg.info.origin.position.x;
    gmap.info.origin_y = gmap_msg.info.origin.position.y;
    gmap.data = gmap_msg.data;
    return gmap;
  }

  nav_msgs::msg::Path make_nav_path(const std_msgs::msg::Header &header, const std::vector<MPPI::vec6_t> &path)
  {
    nav_msgs::msg::Path ppath;
    ppath.header = header;
    std::vector<geometry_msgs::msg::PoseStamped> poses(path.size());
    for (int i = 0; i < (int)path.size(); i++)
    {
      poses.at(i).header = header;
      poses.at(i).pose = make_geometry_pose(path[i](3), path[i](4), 0.1, path[i](5));
    }
    ppath.poses = poses;
    return ppath;
  }

  geometry_msgs::msg::Twist make_twist(const MPPI::vec3_t &v)
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = v(0);
    twist.linear.y = v(1);
    twist.angular.z = v(2);
    return twist;
  }

  visualization_msgs::msg::MarkerArray make_samples_marker(const std_msgs::msg::Header &header, const std::vector<std::vector<MPPI::vec6_t>> &sample_path)
  {
    visualization_msgs::msg::MarkerArray markers;
    size_t sample_num = sample_path.size();
    for (int i = 0; i < (int)sample_num; i++)
    {
      visualization_msgs::msg::Marker marker;
      marker.header = header;
      marker.ns = "sample_path";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.01;
      marker.color.a = 0.3;
      marker.color.r = 0.5;
      marker.color.g = 0.5;
      marker.color.b = 0.5;
      size_t path_num = sample_path[i].size();
      // std::cout << path_num << std::endl;
      for (int j = 0; j < (int)path_num; j++)
      {
        geometry_msgs::msg::Point p;
        p.x = sample_path[i][j](3);
        p.y = sample_path[i][j](4);
        p.z = 0.0;
        marker.points.push_back(p);
      }
      markers.markers.push_back(marker);
    }
    return markers;
  }
  geometry_msgs::msg::Twist stop()
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    return twist;
  }
};