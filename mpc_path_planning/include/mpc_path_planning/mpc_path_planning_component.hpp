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
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <std_msgs/msg/empty.hpp>

#include "common_lib/ros2_utility/extension_msgs_util.hpp"
#include "common_lib/ros2_utility/msg_util.hpp"
#include "common_lib/ros2_utility/ros_opencv_util.hpp"
#include "common_lib/ros2_utility/tf_util.hpp"
#include "extension_node/extension_node.hpp"
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
// ソートの実行方法設定
//    #define SORT_MODE std::execution::seq // 逐次実行
//    #define SORT_MODE std::execution::par // 並列実行
#define SORT_MODE std::execution::par_unseq // 並列化・ベクトル化
// モデル設定
#define NON_HOLONOMIC
// デバック関連設定
#define PLANNING_DEBUG_OUTPUT
// #define MAP_GEN_DEBUG_OUTPUT
#define CONTROL_DEBUG_OUTPUT
//******************************************************************************
using namespace common_lib;
using namespace std::chrono_literals;
class MPCPathPlanning : public ExtensionNode
{
public:
  MPCPathPlanning(const rclcpp::NodeOptions &options) : MPCPathPlanning("", options) {}
  MPCPathPlanning(
      const std::string &name_space = "",
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : ExtensionNode("mpc_path_planning_node", name_space, options),
        tf_buffer_(this->get_clock()),
        listener_(tf_buffer_)
  {
    RCLCPP_INFO(this->get_logger(), "start mpc_path_planning_node");
    // get param
    std::string MAP_TOPIC = param<std::string>("mpc_path_planning.topic_name.map", "/map");
    std::string TARGET_TOPIC =
        param<std::string>("mpc_path_planning.topic_name.target", "/goal_pose");
    std::string ODOM_TOPIC = param<std::string>("mpc_path_planning.topic_name.odom", "/odom");
    std::string INITPATH_TOPIC =
        param<std::string>("mpc_path_planning.topic_name.init_path", "mpc_path_planning/init_path");
    std::string GLOBALPATH_TOPIC =
        param<std::string>("mpc_path_planning.topic_name.global_path", "global_path_planning/path");
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
    mpc_config_.forward_only = param<bool>("mpc_path_planning.mpc.forward_only", false);
    mpc_config_.min_vel = param<double>("mpc_path_planning.mpc.min_vel", 0.0);         // [m/s]
    mpc_config_.max_vel = param<double>("mpc_path_planning.mpc.max_vel", 1.0);         // [m/s]
    mpc_config_.min_angular = param<double>("mpc_path_planning.mpc.min_angular", 0.0); // [rad/s]
    mpc_config_.max_angular = param<double>("mpc_path_planning.mpc.max_angular", 1.0); // [rad/s]
    mpc_config_.max_acc = param<double>("mpc_path_planning.mpc.max_acc", 0.98);        // [m/s^2]
    mpc_config_.max_angular_acc =
        param<double>("mpc_path_planning.mpc.max_angular_acc", 1.0); // [rad/s^2]
    double xy_vel_time_constant = param<double>("mpc_path_planning.mpc.xy_vel_time_constant", 0.0);
    double theta_vel_time_constant =
        param<double>("mpc_path_planning.mpc.theta_vel_time_constant", 0.0);
    std::vector<double> STATE_WEIGHT = param<std::vector<double>>(
        "mpc_path_planning.mpc.weight.state", std::vector<double>{10, 10, 6, 200, 200, 60});
    std::vector<double> FINAL_STATE_WEIGHT = param<std::vector<double>>(
        "mpc_path_planning.mpc.weight.final_state", std::vector<double>{10, 10, 6, 200, 200, 60});
    std::vector<double> REF_STATE_WEIGHT = param<std::vector<double>>(
        "mpc_path_planning.mpc.weight.ref_state", std::vector<double>{0, 0, 0, 0, 0, 0});
    std::vector<double> CONTROL_WEIGHT = param<std::vector<double>>(
        "mpc_path_planning.mpc.weight.control", std::vector<double>{3, 3, 2});
    std::vector<double> DIFF_CONTROL_WEIGHT = param<std::vector<double>>(
        "mpc_path_planning.mpc.weight.diff_control", std::vector<double>{3, 3, 2});
    mpc_config_.state_weight << STATE_WEIGHT[0], STATE_WEIGHT[1], STATE_WEIGHT[2], STATE_WEIGHT[3],
        STATE_WEIGHT[4], STATE_WEIGHT[5];
    mpc_config_.final_state_weight << FINAL_STATE_WEIGHT[0], FINAL_STATE_WEIGHT[1],
        FINAL_STATE_WEIGHT[2], FINAL_STATE_WEIGHT[3], FINAL_STATE_WEIGHT[4], FINAL_STATE_WEIGHT[5];
    mpc_config_.ref_state_weight << REF_STATE_WEIGHT[0], REF_STATE_WEIGHT[1], REF_STATE_WEIGHT[2],
        REF_STATE_WEIGHT[3], REF_STATE_WEIGHT[4], REF_STATE_WEIGHT[5];
    mpc_config_.control_weight << CONTROL_WEIGHT[0], CONTROL_WEIGHT[1], CONTROL_WEIGHT[2];
    mpc_config_.diff_control_weight << DIFF_CONTROL_WEIGHT[0], DIFF_CONTROL_WEIGHT[1],
        DIFF_CONTROL_WEIGHT[2];
    MPCPathPlanner::calc_lpf_gain(mpc_config_, xy_vel_time_constant, theta_vel_time_constant);
    std::cout << "lpf_xy_gain:" << mpc_config_.lpf_xy_gain << std::endl;
    std::cout << "lpf_theta_gain:" << mpc_config_.lpf_theta_gain << std::endl;
    mpc_config_.warm_start_d_target_norm =
        param<double>("mpc_path_planning.mpc.warm_start.diff_target_norm", 0.1);
    mpc_config_.warm_start_d_latest_gpl_norm =
        param<double>("mpc_path_planning.mpc.warm_start.latest_gpl_norm", 0.5);
    mpc_config_.terminal_range = param<double>("mpc_path_planning.mpc.terminal_range", 0.0);
    std::string IPOPT_SB = param<std::string>("mpc_path_planning.mpc.ipopt.sb", "yes");
    std::string IPOPT_LINEAR_SOLVER = param<std::string>(
        "mpc_path_planning.mpc.ipopt.linear_solver", "mumps"); // mumpsは遅い ma27はHSLライブラリ必要
    int IPOPT_MAX_ITER = param<int>("mpc_path_planning.mpc.ipopt.max_iter", 500);
    double IPOPT_ACCEPTABLE_TOL =
        param<double>("mpc_path_planning.mpc.ipopt.acceptable_tol", 0.000001);
    double IPOPT_COMPL_INF_TOL = param<double>("mpc_path_planning.mpc.ipopt.compl_inf_tol", 0.0001);
    auto solver_option = [&]() -> casadi::Dict
    {
      return {
          {"ipopt.sb", IPOPT_SB.c_str()}, // コンソールにヘッダを出力しない
          {"ipopt.linear_solver",
           IPOPT_LINEAR_SOLVER.c_str()}, // mumpsは遅い ma27はHSLライブラリ必要
          {"ipopt.max_iter", IPOPT_MAX_ITER},
          {"ipopt.acceptable_tol", IPOPT_ACCEPTABLE_TOL},
          {"ipopt.compl_inf_tol", IPOPT_COMPL_INF_TOL},
          //{"ipopt.tol", 1.0e-8},
          {"ipopt.print_level", 0},
          {"print_time", false},
          {"ipopt.warm_start_init_point", "yes"},
          // {"ipopt.hessian_approximation", "limited-memory"},//ヘッシアン近似（準ニュートン法）を行い反復一回あたりの計算は早くなる
          {"ipopt.fixed_variable_treatment", "make_constraint"},
          {"expand", true},
      };
    };
    mpc_config_.solver_option = solver_option();
    // mpc_config_.solver_option = MPCPathPlanner::default_solver_option();//default 設定
    // init
    RCLCPP_INFO(this->get_logger(), "Initialization !");
    end_ = true;
    target_pose_ = std::nullopt;
    planner_ = std::make_shared<MPCPathPlanner>(mpc_config_);
    // 追加の制約等設定
    auto init_func = std::bind(&MPCPathPlanning::init_parameter, this);
    auto add_cost_func = std::bind(
        &MPCPathPlanning::add_cost_function, this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3);
    auto add_const_func = std::bind(
        &MPCPathPlanning::add_constraints, this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3);
    auto set_param_func = std::bind(&MPCPathPlanning::set_user_param, this, std::placeholders::_1);
    planner_->set_user_function(init_func, add_cost_func, add_const_func, set_param_func);
    // 最適化時間計測用
    planner_->timer = [&]()
    { return now().seconds(); };
    // モデル設定
#if defined(NON_HOLONOMIC)
    planner_->set_kinematics_model(two_wheeled_model(mpc_config_), false);
#else
    planner_->set_kinematics_model(omni_directional_model(mpc_config_));
#endif
    planner_->init_solver();
    // publisher
    init_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(INITPATH_TOPIC, rclcpp::QoS(10));
    opti_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(OPTIPATH_TOPIC, rclcpp::QoS(10));
    opti_twists_pub_ = this->create_publisher<extension_msgs::msg::TwistMultiArray>(
        OPTITWISTS_TOPIC, rclcpp::QoS(10));
    perfomance_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "mpc_path_planning/solve_time", rclcpp::QoS(5));
    // subscriber
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        TARGET_TOPIC, rclcpp::QoS(10), [&](geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
        target_pose_ = make_pose(msg->pose);
        end_ = false;
        global_path_ = std::nullopt; });
    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        GLOBALPATH_TOPIC, rclcpp::QoS(10).reliable(),
        [&](const nav_msgs::msg::Path::SharedPtr msg)
        { global_path_ = make_path(*msg); });
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        ODOM_TOPIC, rclcpp::QoS(10),
        [&](nav_msgs::msg::Odometry::SharedPtr msg)
        { now_vel_ = make_twist(*msg); });
    end_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "mpc_path_planning/end", rclcpp::QoS(10),
        [&](std_msgs::msg::Empty::SharedPtr msg)
        { end_ = true; });
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        MAP_TOPIC, rclcpp::QoS(10).reliable(),
        [&](const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
        {
          map_ = make_gridmap(*msg);
          planner_->set_map(map_.value());
        });
    // timer
    path_planning_timer_ = this->create_wall_timer(1s * PLANNING_PERIOD, [&]()
                                                   {
      if (end_) {
        target_pose_ = std::nullopt;
        global_path_ = std::nullopt;
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
        // 経路計算
        if (global_path_) {
          planner_->set_grid_path(global_path_.value());
          RCLCPP_INFO_CHANGE(1, this->get_logger(), "get map");
          obstacles_detect(base_link_pose);
          if (target_pose_) {
#if defined(NON_HOLONOMIC)
            now_vel_.linear.y = 0.0;
#endif
#if defined(PLANNING_DEBUG_OUTPUT)
            std::cout << "----------------------------------------------------------" << std::endl;
            std::cout << "now_vel:" << now_vel_ << std::endl;
            std::cout << "end:" << target_pose_.value() << std::endl;
            std::cout << "start:" << base_link_pose << std::endl;
            std::cout << "end:" << target_pose_.value() << std::endl;
#endif
            PathPointd start, end;
            start.pose = base_link_pose;
            start.velocity = now_vel_;
            end.pose = target_pose_.value();
            // path planning
            Pathd opti_path = planner_->path_planning(start, end);
            Pathd init_path = planner_->get_init_path();
            // publish grid path
            init_path_pub_->publish(
              make_nav_path(make_header(MAP_FRAME, rclcpp::Clock().now()), init_path));
            if (planner_->optimization()) {  //最適化成功
              // publish opti path
              auto [o_ppath, o_vpath, o_apath] =
                make_msg_paths(make_header(MAP_FRAME, rclcpp::Clock().now()), opti_path);
              opti_path_pub_->publish(o_ppath);
              opti_twists_pub_->publish(o_vpath);
              //debug
              perfomance_pub_->publish(make_float32(planner_->get_solve_time() * 1000));
            }
          }
        }
      } });
  }

private:
  bool gen_distance_map_;
  bool end_ = false;
  // param
  std::string MAP_FRAME;
  std::string ROBOT_FRAME;
  double PLANNING_PERIOD;
  mpc_config_t mpc_config_;
  double OBSTACLE_DETECT_DIST;
  size_t OBSTACLES_MAX_SIZE;
  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;
  // timer
  rclcpp::TimerBase::SharedPtr path_planning_timer_;
  // subscriber
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr end_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  // publisher
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr init_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr opti_path_pub_;
  rclcpp::Publisher<extension_msgs::msg::TwistMultiArray>::SharedPtr opti_twists_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr perfomance_pub_;
  // twist
  Twistd now_vel_;
  // pose
  std::optional<Pose3d> target_pose_;
  // planner
  std::optional<GridMap> map_;
  std::optional<Pathd> global_path_;
  std::shared_ptr<MPCPathPlanner> planner_;

  std::vector<std::pair<Vector2d, double>> obstacles_;
  void obstacles_detect(Pose3d robot_pose)
  {
    Vector2d robot = {robot_pose.position.x, robot_pose.position.y};
    if (map_)
    {
      obstacles_.clear();
#pragma omp parallel for
      for (size_t y = 0; y < map_.value().info.height; y++)
      {
        for (size_t x = 0; x < map_.value().info.width; x++)
        {
          if (map_.value().is_wall(x, y))
          {
            Vector2d obstacle = map_.value().grid_to_pos(x, y);
            double dist = (obstacle - robot).norm();
            if (dist < OBSTACLE_DETECT_DIST)
            {
#pragma omp critical
              obstacles_.push_back({obstacle, dist});
            }
          }
        }
      }
      if (obstacles_.size() != 0)
      {
        std::sort(
            SORT_MODE, obstacles_.begin(), obstacles_.end(), [](const auto &a, const auto &b)
            { return a.second < b.second; }); // 距離が近い順にソート
        if (obstacles_.size() > OBSTACLES_MAX_SIZE)
          obstacles_.erase(obstacles_.begin() + OBSTACLES_MAX_SIZE, obstacles_.begin() + obstacles_.size());
      }
    }
  }

  // MPU追加制約
  void init_parameter() {}

  void add_cost_function(casadi::MX &cost, const casadi::MX &X, const casadi::MX &U) {}

  void add_constraints(casadi::Opti &opti, const casadi::MX &X, const casadi::MX &U)
  {
    using namespace casadi;
    using Sl = casadi::Slice;

    // const double robot_colision_size = 0.0;
    // for (size_t i = 1; i < X.size2(); i++) {//get_guard_circle_subject(const casadi::MX &xy, const casadi::MX &center, const casadi::MX &size, std::string comp)
    //   // test
    //   opti.subject_to(get_guard_circle_subject(
    //     X(Sl(3, 5), i), MX::vertcat({-1.15, 0.0}),
    //     MX::vertcat({0.05 + robot_colision_size, 0.05 + robot_colision_size}),
    //     "keep out"));
    // }
  }
  void set_user_param(casadi::Opti &opti) {}
};