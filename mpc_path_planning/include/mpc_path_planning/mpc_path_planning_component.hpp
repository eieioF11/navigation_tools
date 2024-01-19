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
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/empty.hpp>
// opencv
#include <opencv2/opencv.hpp>

#include "common_lib/ros2_utility/extension_msgs_util.hpp"
#include "common_lib/ros2_utility/msg_util.hpp"
#include "common_lib/ros2_utility/tf_util.hpp"
#include "cv_bridge/cv_bridge.h"
#include "extension_node/extension_node.hpp"
// #include "common_lib/ros2_utility/marker_util.hpp"
// other
#include "common_lib/common_lib.hpp"

#define PLANNER_DEBUG_OUTPUT
// grid path planner
#include "common_lib/planner/a_star.hpp"
#include "common_lib/planner/dijkstra.hpp"
#include "common_lib/planner/extension_a_star.hpp"
#include "common_lib/planner/wave_propagation.hpp"
// mpc
#include "common_lib/planner/mpc_path_planner.hpp"

#define _ENABLE_ATOMIC_ALIGNMENT_FIX
//******************************************************************************
#define NON_HOLONOMIC
// デバック関連設定
#define PLANNING_DEBUG_OUTPUT
// #define CONTROL_DEBUG_OUTPUT
//******************************************************************************
using namespace common_lib;
using namespace std::chrono_literals;
class MPCPathPlanning : public ExtensionNode
{
public:
  MPCPathPlanning(const rclcpp::NodeOptions & options) : MPCPathPlanning("", options) {}
  MPCPathPlanning(
    const std::string & name_space = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : ExtensionNode("mpc_path_planning_node", name_space, options),
    tf_buffer_(this->get_clock()),
    listener_(tf_buffer_)
  {
    RCLCPP_INFO(this->get_logger(), "start mpc_path_planning_node");
    // get param
    std::string MAP_TOPIC = param<std::string>("mpc_path_planning.topic_name.map", "/map");
    std::string TARGET_TOPIC =
      param<std::string>("mpc_path_planning.topic_name.target", "/goal_pose");
    std::string GRIDPATH_TOPIC =
      param<std::string>("mpc_path_planning.topic_name.grid_path", "mpc_path_planning/grid_path");
    std::string OPTIPATH_TOPIC =
      param<std::string>("mpc_path_planning.topic_name.opti_path", "mpc_path_planning/opti_path");
    std::string OPTITWISTS_TOPIC =
      param<std::string>("mpc_path_planning.topic_name.opti_twists", "mpc_path_planning/twists");
    // frame
    MAP_FRAME = param<std::string>("mpc_path_planning.tf_frame.map_frame", "map");
    ROBOT_FRAME = param<std::string>("mpc_path_planning.tf_frame.robot_frame", "base_link");
    // setup
    PLANNING_PERIOD = param<double>("mpc_path_planning.planning_period", 0.001);
    // mpc
    mpc_config_.dt = param<double>("mpc_path_planning.mpc.dt", 0.06);
    double HORIZON_TIME = param<double>("mpc_path_planning.mpc.horizon_time", 7.0);
    mpc_config_.horizon = static_cast<size_t>(HORIZON_TIME / mpc_config_.dt);
    mpc_config_.pos_error = param<double>("mpc_path_planning.mpc.pos_error", 100.0);
    mpc_config_.max_vel = param<double>("mpc_path_planning.mpc.max_vel", 1.0);          // [m/s]
    mpc_config_.max_angular = param<double>("mpc_path_planning.mpc.max_angular", 1.0);  // [m/s]
    mpc_config_.max_acc = param<double>("mpc_path_planning.mpc.max_acc", 0.98);         // [m/s^2]
    mpc_config_.max_angular_acc =
      param<double>("mpc_path_planning.mpc.max_angular_acc", 1.0);  // [rad/s^2]
    std::vector<double> STATE_WEIGHT = param<std::vector<double>>(
      "mpc_path_planning.mpc.weight.state", std::vector<double>{10, 10, 6, 200, 200, 60});
    std::vector<double> FINAL_STATE_WEIGHT = param<std::vector<double>>(
      "mpc_path_planning.mpc.weight.final_state", std::vector<double>{10, 10, 6, 200, 200, 60});
    std::vector<double> CONTROL_WEIGHT = param<std::vector<double>>(
      "mpc_path_planning.mpc.weight.control", std::vector<double>{3, 3, 2});
    mpc_config_.state_weight << STATE_WEIGHT[0], STATE_WEIGHT[1], STATE_WEIGHT[2], STATE_WEIGHT[3],
      STATE_WEIGHT[4], STATE_WEIGHT[5];
    mpc_config_.final_state_weight << FINAL_STATE_WEIGHT[0], FINAL_STATE_WEIGHT[1],
      FINAL_STATE_WEIGHT[2], FINAL_STATE_WEIGHT[3], FINAL_STATE_WEIGHT[4], FINAL_STATE_WEIGHT[5];
    mpc_config_.control_weight << CONTROL_WEIGHT[0], CONTROL_WEIGHT[1], CONTROL_WEIGHT[2];
    mpc_config_.lpf_xy_gain = mpc_config_.dt / (mpc_config_.dt + mpc_config_.xy_vel_time_constant);
    mpc_config_.lpf_theta_gain =
      mpc_config_.dt / (mpc_config_.dt + mpc_config_.theta_vel_time_constant);
    std::string IPOPT_SB = param<std::string>("mpc_path_planning.mpc.ipopt.sb", "yes");
    std::string IPOPT_LINEAR_SOLVER = param<std::string>(
      "mpc_path_planning.mpc.ipopt.linear_solver", "mumps");  // mumpsは遅い ma27はHSLライブラリ必要
    int IPOPT_MAX_ITER = param<int>("mpc_path_planning.mpc.ipopt.max_iter", 500);
    double IPOPT_ACCEPTABLE_TOL =
      param<double>("mpc_path_planning.mpc.ipopt.acceptable_tol", 0.000001);
    double IPOPT_COMPL_INF_TOL = param<double>("mpc_path_planning.mpc.ipopt.compl_inf_tol", 0.0001);
    auto solver_option = [&]() -> casadi::Dict {
      return {
        {"ipopt.sb", IPOPT_SB.c_str()},  // コンソールにヘッダを出力しない
        // {"ipopt.linear_solver", "ma27"}, // mumpsは遅い ma27はHSLライブラリ必要
        {"ipopt.linear_solver",
         IPOPT_LINEAR_SOLVER.c_str()},  // mumpsは遅い ma27はHSLライブラリ必要
        {"ipopt.max_iter", IPOPT_MAX_ITER},
        {"ipopt.acceptable_tol", IPOPT_ACCEPTABLE_TOL},
        {"ipopt.compl_inf_tol", IPOPT_COMPL_INF_TOL},
        //{"ipopt.tol", 1.0e-8},
        {"ipopt.print_level", 0},
        {"print_time", false},
        {"ipopt.warm_start_init_point", "yes"},
        // {"ipopt.hessian_approximation", "limited-memory"},//使えてない
        {"ipopt.fixed_variable_treatment", "make_constraint"},
        {"expand", true},
      };
    };
    mpc_config_.solver_option = solver_option();
    // mpc_config_.solver_option = MPCPathPlanner::default_solver_option();//default 設定
    std::string GRID_PATH_PLANNING =
      param<std::string>("mpc_path_planning.mpc.grid_path_planner", "a_star");
    // init
    RCLCPP_INFO(this->get_logger(), "Initialization !");
    end_ = true;
    target_pose_ = std::nullopt;
    pre_planning_time_ = this->get_clock()->now();
    planner_ = std::make_shared<MPCPathPlanner>(mpc_config_);
    // 追加の制約等設定
    auto init_func = std::bind(&MPCPathPlanning::init_parameter, this);
    auto add_cost_func = std::bind(
      &MPCPathPlanning::add_cost_function, this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3);
    auto add_const = std::bind(
      &MPCPathPlanning::add_constraints, this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3);
    // planner_->set_user_function(init_func,add_cost_func,add_const);
    // 最適化時間計測用
    planner_->timer = [&]() { return now().seconds(); };
    // グリッドパスプランニング設定
    if (GRID_PATH_PLANNING.compare("wave_propagation") == 0)
      planner_->set_grid_path_planner(std::make_shared<WavePropagation>());
    else if (GRID_PATH_PLANNING.compare("wave_propagation_dist_map") == 0)
      planner_->set_grid_path_planner(std::make_shared<WavePropagation>(true));
    else if (GRID_PATH_PLANNING.compare("a_star") == 0)
      planner_->set_grid_path_planner(std::make_shared<AStar>());
    else if (GRID_PATH_PLANNING.compare("extension_a_star") == 0)
      planner_->set_grid_path_planner(std::make_shared<ExtensionAStar>());
    else if (GRID_PATH_PLANNING.compare("dijkstra") == 0)
      planner_->set_grid_path_planner(std::make_shared<Dijkstra>());
    else if (GRID_PATH_PLANNING.compare("dijkstra_dist_map") == 0)
      planner_->set_grid_path_planner(std::make_shared<Dijkstra>(true));
    else
      RCLCPP_WARN(this->get_logger(), "grid path planning not used");
    RCLCPP_INFO(this->get_logger(), "grid path planning: %s", GRID_PATH_PLANNING.c_str());
    // モデル設定
#if defined(NON_HOLONOMIC)
    planner_->set_kinematics_model(two_wheeled_model(mpc_config_), false);
#else
    planner_->set_kinematics_model(omni_directional_model(mpc_config_));
#endif
    planner_->init_solver();
    // publisher
    grid_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(GRIDPATH_TOPIC, rclcpp::QoS(10));
    opti_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(OPTIPATH_TOPIC, rclcpp::QoS(10));
    opti_twists_pub_ = this->create_publisher<extension_msgs::msg::TwistMultiArray>(
      OPTITWISTS_TOPIC, rclcpp::QoS(10));
    perfomance_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "mpc_path_planning/solve_time", rclcpp::QoS(5));
    dist_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "mpc_path_planning/dist_map", rclcpp::QoS(5));
    // subscriber
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      MAP_TOPIC, rclcpp::QoS(10).reliable(),
      [&](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map_msg_ = msg;
        gen_distance_map_ = false;
      });
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      TARGET_TOPIC, rclcpp::QoS(10), [&](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        target_pose_ = make_pose(msg->pose);
        end_ = false;
      });
    end_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "mpc_path_planning/end", rclcpp::QoS(10),
      [&](std_msgs::msg::Empty::SharedPtr msg) { end_ = true; });
    // timer
    path_planning_timer_ = this->create_wall_timer(1s * PLANNING_PERIOD, [&]() {
      if (end_) {
        target_pose_ = std::nullopt;
      }
      if (!tf_buffer_.canTransform(
            ROBOT_FRAME, MAP_FRAME, rclcpp::Time(0),
            tf2::durationFromSec(1.0))) {  // 変換無いよ
        RCLCPP_WARN(
          this->get_logger(), "%s %s can not Transform", MAP_FRAME.c_str(), ROBOT_FRAME.c_str());
        return;
      }
      auto map_to_base_link = lookup_transform(tf_buffer_, ROBOT_FRAME, MAP_FRAME);
      if (map_to_base_link) {
        Pose3d base_link_pose = make_pose(map_to_base_link.value().transform);
        RCLCPP_INFO_CHANGE(0, this->get_logger(), "get base_link pose");
        // 速度計算
        auto now_time = this->get_clock()->now();
        double dt = (now_time - pre_planning_time_).seconds();
        now_vel_.linear = (base_link_pose.position - pre_base_link_pose_.position) * dt;
#if defined(NON_HOLONOMIC)
        now_vel_.linear.x = std::hypot(now_vel_.linear.x, now_vel_.linear.y);
        now_vel_.linear.y = 0.0;
#endif
        now_vel_.angular =
          Angles(base_link_pose.orientation.get_rpy() - pre_base_link_pose_.orientation.get_rpy()) *
          dt;
        pre_base_link_pose_ = base_link_pose;
        pre_planning_time_ = this->get_clock()->now();
        // 経路計算
        if (map_msg_) {
          calc_distance_map();
          planner_->set_map(make_gridmap(dist_map_msg_));
          RCLCPP_INFO_CHANGE(1, this->get_logger(), "get map");
          if (target_pose_) {
#if defined(PLANNING_DEBUG_OUTPUT)
            std::cout << "----------------------------------------------------------" << std::endl;
            std::cout << "start:" << base_link_pose << std::endl;
            std::cout << "end:" << target_pose_.value() << std::endl;
#endif
            PathPointd start, end;
            start.pose = base_link_pose;
            start.velocity = now_vel_;
            end.pose = target_pose_.value();
            // path planning
            Pathd opti_path = planner_->pathplanning(start, end);
            Pathd grid_path = planner_->get_grid_path();
            // publish grid path
            grid_path_pub_->publish(
              make_nav_path(make_header(MAP_FRAME, rclcpp::Clock().now()), grid_path));
            if (planner_->optimization()) {  //最適化成功
              // publish opti path
              auto [o_ppath, o_vpath, o_apath] =
                make_msg_paths(make_header(MAP_FRAME, rclcpp::Clock().now()), opti_path);
              opti_path_pub_->publish(o_ppath);
              opti_twists_pub_->publish(o_vpath);
              //debug
              perfomance_pub_->publish(make_float32(planner_->solve_time_ * 1000));
            }
          }
        }
      }
    });
  }

private:
  bool gen_distance_map_;
  bool end_ = false;
  // param
  std::string MAP_FRAME;
  std::string ROBOT_FRAME;
  double PLANNING_PERIOD;
  mpc_config_t mpc_config_;
  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;
  // timer
  rclcpp::TimerBase::SharedPtr path_planning_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Time pre_planning_time_;
  // subscriber
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr end_sub_;
  // publisher
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr grid_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr opti_path_pub_;
  rclcpp::Publisher<extension_msgs::msg::TwistMultiArray>::SharedPtr opti_twists_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr perfomance_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr dist_map_pub_;
  // twist
  Twistd now_vel_;
  // pose
  std::optional<Pose3d> target_pose_;
  Pose3d pre_base_link_pose_;
  // map
  nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
  nav_msgs::msg::OccupancyGrid dist_map_msg_;
  // planner
  std::shared_ptr<MPCPathPlanner> planner_;

  Vector2d vector2_cast(const PathPointd & p) { return {p.pose.position.x, p.pose.position.y}; }
  PathPointd pathpoint_cast(const Vector2d & v)
  {
    PathPointd p;
    p.pose.position.x = v.x;
    p.pose.position.y = v.y;
    p.pose.position.z = 0.0;
    return p;
  }

  Vector3d Angles(Vector3d in)
  {
    Vector3d out;
    out.x = Angle(in.x);
    out.y = Angle(in.y);
    out.z = Angle(in.z);
    return out;
  }

  void calc_distance_map()
  {
    if (!map_msg_) return;
    if (gen_distance_map_) return;
    using namespace cv;
#if defined(PLANNING_DEBUG_OUTPUT)
    std::cout << "calc distance map start!" << std::endl;
#endif
    dist_map_msg_.header = map_msg_->header;
    dist_map_msg_.info = map_msg_->info;
    dist_map_msg_.data.resize(dist_map_msg_.info.width * dist_map_msg_.info.height);
    Pose3d origin = make_pose(map_msg_->info.origin);
    Mat img = Mat::zeros(cv::Size(map_msg_->info.width, map_msg_->info.height), CV_8UC1);
    for (unsigned int y = 0; y < map_msg_->info.height; y++) {
      for (unsigned int x = 0; x < map_msg_->info.width; x++) {
        unsigned int i = x + (map_msg_->info.height - y - 1) * map_msg_->info.width;
        int intensity = 205;
        if (map_msg_->data[i] >= 0 && map_msg_->data[i] <= 100)
          intensity = std::round((float)(100.0 - map_msg_->data[i]) * 2.55);
        img.at<unsigned char>(y, x) = intensity;
      }
    }
    Mat binary, dist_img, convert_dist_img;
    //二値化
    cv::threshold(img, binary, 250, 255, cv::THRESH_BINARY);
    //距離場計算
    cv::distanceTransform(binary, dist_img, CV_DIST_L2, 5);
    // dist_img.convertTo(convert_dist_img, CV_32FC1, 1.0 / 255.0);
    dist_img.convertTo(convert_dist_img, CV_32FC1, 1.0);
    double min, max;
    cv::Point min_p, max_p;
    cv::minMaxLoc(convert_dist_img, &min, &max, &min_p, &max_p);
    // std::cout << max << std::endl;
    for (size_t y = 0; y < dist_map_msg_.info.height; y++) {
      for (size_t x = 0; x < dist_map_msg_.info.width; x++) {
        float pixel =
          transform_range<float>(max - convert_dist_img.at<float>(y, x), min, max, 0.0, 100.0);
        if (pixel > 100.0) pixel = 100.0;
        if (pixel < 0.0) pixel = 0.0;
        // std::cout << (float)pixel << std::endl;
        dist_map_msg_.data[dist_map_msg_.info.width * (dist_map_msg_.info.height - y - 1) + x] =
          static_cast<int8_t>(pixel);
      }
    }
    dist_map_pub_->publish(dist_map_msg_);
    // cv::resize(img, convert_mat, cv::Size(), ratio, ratio, cv::INTER_NEAREST);
    // cv::imshow("map", convert_mat);
    // cv::resize(binary, convert_mat, cv::Size(), ratio, ratio, cv::INTER_NEAREST);
    // cv::imshow("binary_map", convert_mat);
    // dist_img = dist_img * 51.0;
    // Mat convert_mat;
    // //縦横どっちか長い方は？
    // int big_width = img.cols > img.rows ? img.cols : img.rows;
    // //割合
    // int width = 1000;
    // double ratio = ((double)width / (double)big_width);
    // cv::resize(dist_img, convert_mat, cv::Size(), ratio, ratio, cv::INTER_NEAREST);
    // cv::imshow("dist_map", convert_mat);
    // cv::waitKey(0);
#if defined(PLANNING_DEBUG_OUTPUT)
    std::cout << "calc distance map end!" << std::endl;
#endif
    gen_distance_map_ = true;
  }

  // MPU追加制約
  void init_parameter()
  {
    // for (size_t i = 0; i < lagori_poses_mx.size(); i++) {
    //   lagori_poses_mx[i] = add_parameter(2, 1, [this, i]() {
    //     using namespace casadi;
    //     DM dm_pos = DM::zeros(2);
    //     std::copy(
    //       lagori_poses[i].data(), lagori_poses[i].data() + lagori_poses[i].size(), dm_pos.ptr());

    //     return dm_pos;
    //   });
    // }
  }

  void add_cost_function(casadi::MX & cost, const casadi::MX & X, const casadi::MX & U)
  {
    using namespace casadi;
    using Sl = casadi::Slice;

    // const double robot_colision_size = 0.8;
    // const double fence_width = 0.05;

    // for(size_t i = 1; i < X.size2(); i++)
    // {
    //     cost = cost + 100*get_guard_circle_cost(X(Sl(3,5), i), MX::vertcat({6-fence_width, 6-fence_width}), MX::vertcat({0.5+robot_colision_size-0.1, 0.5+robot_colision_size-0.1}), "keep out");
    // }
  }

  void add_constraints(casadi::Opti & opti, const casadi::MX & X, const casadi::MX & U)
  {
    using namespace casadi;
    using Sl = casadi::Slice;

    const double robot_colision_size = 0.75;

    // for (size_t i = 1; i < X.size2(); i++) {//get_guard_circle_subject(const casadi::MX &xy, const casadi::MX &center, const casadi::MX &size, std::string comp)
    //   // test
    //   opti.subject_to(get_guard_circle_subject(
    //     X(Sl(3, 5), i), MX::vertcat({-1.15, 0.0}),
    //     MX::vertcat({0.05 + robot_colision_size, 0.05 + robot_colision_size}),
    //     "keep out"));
    // }

    // const double fence_width = 0.05;
    // // シーカー
    // for (size_t i = 1; i < X.size2(); i++) {
    //   // フィールド・ラゴリエリア内にいる制約
    //   opti.subject_to(get_guard_rect_subject(
    //     X(Sl(3, 5), i), 6 - fence_width, 9.5 / 2, 9 - robot_colision_size,
    //     9.5 - robot_colision_size, "keep in"));

    //   // ラゴリ台にぶつからない制約
    //   opti.subject_to(get_guard_rect_subject_approx(
    //     X(Sl(3, 5), i), MX::vertcat({6 - fence_width, 6 - fence_width}),
    //     MX::vertcat({0.5 + robot_colision_size - 0.1, 0.5 + robot_colision_size - 0.1}), 10,
    //     "keep out"));

    //   // R1にぶつからない制約
    //   opti.subject_to(get_guard_rect_subject_approx(
    //     X(Sl(3, 5), i), MX::vertcat({6 - fence_width, 2 - fence_width}),
    //     MX::vertcat({1.3 + robot_colision_size, 1.3 + robot_colision_size}), 6, "keep out"));

    //   // ラゴリにぶつからない制約
    //   for (size_t lagori_num = 0; lagori_num < lagori_poses_mx.size(); lagori_num++) {
    //     double diameter = 0.5 + robot_colision_size - (0.075 * i);
    //     opti.subject_to(get_guard_circle_subject(
    //       X(Sl(3, 5), i), *(lagori_poses_mx[lagori_num]), MX::vertcat({diameter, diameter}),
    //       "keep out"));
    //   }
    // }
  }
};