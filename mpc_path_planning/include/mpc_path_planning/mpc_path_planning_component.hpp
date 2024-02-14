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
// common_lib
#define USE_OMP
#include "common_lib/common_lib.hpp"

// common_lib utility
#include "common_lib/ros2_utility/extension_msgs_util.hpp"
#include "common_lib/ros2_utility/marker_util.hpp"
#include "common_lib/ros2_utility/msg_util.hpp"
#include "common_lib/ros2_utility/ros_opencv_util.hpp"
#include "common_lib/ros2_utility/tf_util.hpp"
#include "extension_node/extension_node.hpp"

// planner
#define PLANNER_DEBUG_OUTPUT
#include "common_lib/planner/mpc_path_planner.hpp"

#define _ENABLE_ATOMIC_ALIGNMENT_FIX
//******************************************************************************
#define EXECUSION_POINT_VALUE 1000000000.0
// #define EXECUSION_POINT_VALUE std::numeric_limits<double>::infinity()
// モデル設定
#define NON_HOLONOMIC
// デバック関連設定
// #define PLANNING_DEBUG_OUTPUT
// #define MAP_GEN_DEBUG_OUTPUT
// #define CONTROL_DEBUG_OUTPUT
// #define USE_IMU_DEBUG
// #define OBSTACLE_DETECT_DEBUG_OUTPUT
//******************************************************************************
using namespace common_lib;
using namespace std::chrono_literals;
class MPCPathPlanning : public ExtensionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using ClientGoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using ServerGoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

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
    GLOBAL_PLANNING_PERIOD = param<double>("mpc_path_planning.global_planning_period", 0.001);
    // mpc
    mpc_config_.dt = param<double>("mpc_path_planning.mpc.dt", 0.06);
    double HORIZON_TIME = param<double>("mpc_path_planning.mpc.horizon_time", 7.0);
    mpc_config_.horizon = static_cast<size_t>(HORIZON_TIME / mpc_config_.dt);
    mpc_config_.pos_error = param<double>("mpc_path_planning.mpc.pos_error", 100.0);
    mpc_config_.forward_only = param<bool>("mpc_path_planning.mpc.forward_only", false);
    mpc_config_.min_vel = param<double>("mpc_path_planning.mpc.min_vel", 0.0);          // [m/s]
    mpc_config_.max_vel = param<double>("mpc_path_planning.mpc.max_vel", 1.0);          // [m/s]
    mpc_config_.min_angular = param<double>("mpc_path_planning.mpc.min_angular", 0.0);  // [rad/s]
    mpc_config_.max_angular = param<double>("mpc_path_planning.mpc.max_angular", 1.0);  // [rad/s]
    mpc_config_.max_acc = param<double>("mpc_path_planning.mpc.max_acc", 0.98);         // [m/s^2]
    mpc_config_.max_angular_acc =
      param<double>("mpc_path_planning.mpc.max_angular_acc", 1.0);  // [rad/s^2]
    double xy_vel_time_constant = param<double>("mpc_path_planning.mpc.xy_vel_time_constant", 0.0);
    double theta_vel_time_constant =
      param<double>("mpc_path_planning.mpc.theta_vel_time_constant", 0.0);
    mpc_config_.state_weight = make_eigen_vector6(param<std::vector<double>>(
      "mpc_path_planning.mpc.weight.state", std::vector<double>{10, 10, 6, 200, 200, 60}));
    mpc_config_.final_state_weight = make_eigen_vector6(param<std::vector<double>>(
      "mpc_path_planning.mpc.weight.final_state", std::vector<double>{10, 10, 6, 200, 200, 60}));
    mpc_config_.ref_state_weight = make_eigen_vector6(param<std::vector<double>>(
      "mpc_path_planning.mpc.weight.ref_state", std::vector<double>{0, 0, 0, 0, 0, 0}));
    mpc_config_.control_weight = make_eigen_vector3(param<std::vector<double>>(
      "mpc_path_planning.mpc.weight.control", std::vector<double>{3, 3, 2}));
    mpc_config_.diff_control_weight = make_eigen_vector3(param<std::vector<double>>(
      "mpc_path_planning.mpc.weight.diff_control", std::vector<double>{3, 3, 2}));
    MPCPathPlanner::calc_lpf_gain(mpc_config_, xy_vel_time_constant, theta_vel_time_constant);
    std::cout << "lpf_xy_gain:" << mpc_config_.lpf_xy_gain << std::endl;
    std::cout << "lpf_theta_gain:" << mpc_config_.lpf_theta_gain << std::endl;
    mpc_config_.warm_start_latest_gpl_norm =
      param<double>("mpc_path_planning.mpc.warm_start.latest_gpl_norm", 0.5);
    mpc_config_.warm_start_latest_target_norm =
      param<double>("mpc_path_planning.mpc.warm_start.latest_target_norm", 0.01);
    mpc_config_.terminal_range = param<double>("mpc_path_planning.mpc.terminal_range", 0.0);
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
        {"ipopt.linear_solver",
         IPOPT_LINEAR_SOLVER.c_str()},  // mumpsは遅い ma27はHSLライブラリ必要
        {"ipopt.max_iter", IPOPT_MAX_ITER},
        {"ipopt.acceptable_tol", IPOPT_ACCEPTABLE_TOL},
        {"ipopt.compl_inf_tol", IPOPT_COMPL_INF_TOL},
        //{"ipopt.tol", 1.0e-8},
        {"ipopt.print_level", 0},
        {"print_time", false},
        {"ipopt.warm_start_init_point", "yes"},
        // {"ipopt.hessian_approximation", "limited-memory"},//ヘッシアン近似（準ニュートン法）を行い反復一回あたりの計算は早くなる
        {"ipopt.fixed_variable_treatment",
         "make_constraint"},  // 固定変数をどのように処理するか  make_constraint:変数を固定する等価制約を追加
        {"expand", true},
      };
    };
    mpc_config_.solver_option = solver_option();
    // mpc_config_.solver_option = MPCPathPlanner::default_solver_option();//default 設定
    OBSTACLE_DETECT_DIST = param<double>("mpc_path_planning.obstacle_detect.dist", 5.0);
    MAX_OBSTACLE_SIZE = param<double>("mpc_path_planning.obstacle_detect.max_obstacle_size", 0.005);
    MIN_OBSTACLE_SIZE = param<double>("mpc_path_planning.obstacle_detect.min_obstacle_size", 0.005);
    OBSTACLES_MAX_SIZE =
      static_cast<size_t>(param<int>("mpc_path_planning.obstacle_detect.list_size", 5));
    NEARBY_OBSTACLE_LIMIT =
      param<double>("mpc_path_planning.obstacle_detect.nearby_obstacle_limit", 0.8);
    // init
    RCLCPP_INFO(this->get_logger(), "Initialization !");
    HALF_OBSTACLE_DETECT_DIST = OBSTACLE_DETECT_DIST / 2.0;
    target_pose_ = std::nullopt;
    global_path_ = std::nullopt;
    controller_running_ = false;
    global_planner_running_ = false;
    end_ = false;
    obstacles_marker_.markers.resize(OBSTACLES_MAX_SIZE);
    planner_ = std::make_shared<MPCPathPlanner>(mpc_config_);
    // 追加の制約等設定
    auto init_func = std::bind(&MPCPathPlanning::init_parameter, this, std::placeholders::_1);
    auto add_cost_func = std::bind(
      &MPCPathPlanning::add_cost_function, this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3);
    auto add_const_func = std::bind(
      &MPCPathPlanning::add_constraints, this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3);
    auto set_param_func = std::bind(&MPCPathPlanning::set_user_param, this, std::placeholders::_1);
    planner_->set_user_function(init_func, add_cost_func, add_const_func, set_param_func);
    // 最適化時間計測用
    planner_->set_timer([&]() { return now().seconds(); });
    // モデル設定
#if defined(NON_HOLONOMIC)
    planner_->set_kinematics_model(two_wheeled_model(mpc_config_), false);
#else
    planner_->set_kinematics_model(omni_directional_model(mpc_config_));
#endif
    planner_->init_solver();
    // publisher
    init_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(INITPATH_TOPIC, rclcpp::QoS(10));
    opti_path_pub_ =
      this->create_publisher<nav_msgs::msg::Path>(OPTIPATH_TOPIC, rclcpp::QoS(10).reliable());
    opti_twists_pub_ = this->create_publisher<extension_msgs::msg::TwistMultiArray>(
      OPTITWISTS_TOPIC, rclcpp::QoS(10).reliable());
    perfomance_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "mpc_path_planning/solve_time", rclcpp::QoS(5));
    perfomance_ave_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "mpc_path_planning/ave_solve_time", rclcpp::QoS(5));
    obstacles_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "mpc_path_planning/obstacles", rclcpp::QoS(5));
    mpc_dt_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "mpc_path_planning/dt", rclcpp::QoS(10).reliable());
    target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "mpc_path_planning/target", rclcpp::QoS(10));
    // subscriber
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      TARGET_TOPIC, rclcpp::QoS(10), [&](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = *msg;
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.feedback_callback =
          [&](
            ClientGoalHandleNavigateToPose::SharedPtr,
            const std::shared_ptr<const NavigateToPose::Feedback> feedback) {};
        send_goal_options.result_callback =
          [&](const ClientGoalHandleNavigateToPose::WrappedResult & result) {};
        planning_action_client_->async_send_goal(goal_msg, send_goal_options);
      });
    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      GLOBALPATH_TOPIC, rclcpp::QoS(10).reliable(), [&](const nav_msgs::msg::Path::SharedPtr msg) {
        global_path_ = make_path(*msg);
        planner_->set_grid_path(global_path_.value());
      });
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      ODOM_TOPIC, rclcpp::QoS(10), [&](nav_msgs::msg::Odometry::SharedPtr msg) {
        now_vel_ = make_twist(*msg);
#if defined(NON_HOLONOMIC)
        now_vel_.linear.y = 0.0;
#endif
      });
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      MAP_TOPIC, rclcpp::QoS(10).reliable(),
      [&](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map_ = make_gridmap(*msg);
        if (!target_pose_) {  //デバック用
          auto map_to_base_link = lookup_transform(tf_buffer_, ROBOT_FRAME, MAP_FRAME);
          if (map_to_base_link) {
            Pose3d base_link_pose = make_pose(map_to_base_link.value().transform);
            obstacles_detect(base_link_pose);
            set_obstacle(make_color(1.0, 1.0, 0.0, 0.1));
            obstacles_pub_->publish(obstacles_marker_);
          }
        }
      });
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10), [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
        log(
          now_vel_.linear.x, now_vel_.linear.y, now_vel_.angular.z, msg->linear.x, msg->linear.y,
          msg->angular.z);
      });
    // action server
    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<NavigateToPose>(
      this, "navigate_to_pose", std::bind(&MPCPathPlanning::handle_goal, this, _1, _2),
      std::bind(&MPCPathPlanning::handle_cancel, this, _1),
      std::bind(&MPCPathPlanning::handle_accepted, this, _1));
    // action client
    control_action_client_ =
      rclcpp_action::create_client<NavigateToPose>(this, "mpc_path_planning/control");
    global_planning_action_client_ =
      rclcpp_action::create_client<NavigateToPose>(this, "mpc_path_planning/global_planning");
    planning_action_client_ =
      rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    // timer
    init_data_logger({"odom_vx", "odom_vy", "odom_w", "u_vx", "u_vy", "u_w"});
    // action serverの起動待機
    while ((!control_action_client_->wait_for_action_server() &&
            !global_planning_action_client_->wait_for_action_server()) &&
           rclcpp::ok()) {
      RCLCPP_INFO(get_logger(), "Waiting for action server...");
      rclcpp::sleep_for(500ms);
    }
  };

  // Action Server
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const NavigateToPose::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "MPCPathPlanner Received goal request");
    if (planner_running_) {
      RCLCPP_WARN(this->get_logger(), "MPCPathPlanner is already running");
      target_pose_ = std::nullopt;
      global_path_ = std::nullopt;
      controller_running_ = false;
      planner_stop_ = true;
      global_planner_running_ = false;
    }
    target_pose_ = make_pose(goal->pose.pose);
    std::cout << "target:" << target_pose_.value() << std::endl;
    planner_->reset();
    controler_send_target(target_pose_.value());
    global_planner_send_target(target_pose_.value());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<ServerGoalHandleNavigateToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "MPCPathPlanner Received request to cancel goal");
    (void)goal_handle;
    target_pose_ = std::nullopt;
    global_path_ = std::nullopt;
    controller_running_ = false;
    global_planner_running_ = false;
    planner_running_ = false;
    planner_->reset();
    control_action_client_->async_cancel_all_goals();
    global_planning_action_client_->async_cancel_all_goals();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<ServerGoalHandleNavigateToPose> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&MPCPathPlanning::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<ServerGoalHandleNavigateToPose> goal_handle)
  {
    rclcpp::Rate loop_rate(1ns);
    rclcpp::Rate wait_loop_rate(10ms);
    Timerd global_planner_timer, mpc_planner_timer;
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    auto result = std::make_shared<NavigateToPose::Result>();
    while (planner_running_) {
      RCLCPP_INFO(this->get_logger(), "Waiting for planner to be ready");
      wait_loop_rate.sleep();
    }
    RCLCPP_INFO(this->get_logger(), "start mpc_path_planning");
    planner_running_ = true;
    while (rclcpp::ok() && planner_running_) {
      if (end_) {
        target_pose_ = std::nullopt;
        global_path_ = std::nullopt;
        controller_running_ = false;
        global_planner_running_ = false;
        planner_running_ = false;
        planner_->reset();
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        return;
      }
      if (planner_stop_) {
        planner_running_ = false;
        planner_stop_ = false;
        RCLCPP_WARN(get_logger(), "planner stopped");
        return;
      }
      if (global_planner_timer.cyclic(GLOBAL_PLANNING_PERIOD)) {
        if (target_pose_) global_planner_send_target(target_pose_.value());
      }
      if (mpc_planner_timer.cyclic(PLANNING_PERIOD)) path_planning(feedback);
      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }
    planner_running_ = false;
    goal_handle->abort(result);
  }

  void path_planning(const NavigateToPose::Feedback::SharedPtr feedback)
  {
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
      obstacles_detect(base_link_pose);
      RCLCPP_INFO_CHANGE(0, this->get_logger(), "get base_link pose");
      // 経路計算
      if (target_pose_) {
#if defined(PLANNING_DEBUG_OUTPUT)
        std::cout << "----------------------------------------------------------" << std::endl;
        std::cout << "now_vel:" << now_vel_ << std::endl;
        std::cout << "end:" << target_pose_.value() << std::endl;
        std::cout << "start:" << base_link_pose << std::endl;
        std::cout << "end:" << target_pose_.value() << std::endl;
#endif
        publish_target(base_link_pose);
        PathPointd start = make_pathpoint<double>(base_link_pose, now_vel_);
        PathPointd end = make_pathpoint<double>(target_pose_.value());
        // path planning
        Pathd opti_path = planner_->path_planning(start, end);
        Pathd init_path = planner_->get_init_path();
        // publish grid path
        init_path_pub_->publish(
          make_nav_path(make_header(MAP_FRAME, rclcpp::Clock().now()), init_path));
        if (planner_->optimization()) {  // 最適化成功
          // publish opti path
          auto [o_ppath, o_vpath, o_apath] =
            make_msg_paths(make_header(MAP_FRAME, rclcpp::Clock().now()), opti_path);
          mpc_dt_pub_->publish(make_float32(mpc_config_.dt));
          opti_path_pub_->publish(o_ppath);
          opti_twists_pub_->publish(o_vpath);
          // debug
          static long double sum = 0.0;
          static long count = 1;
          sum += planner_->get_solve_time();
          double ave_solve_time = sum / count;
          count++;
          perfomance_ave_pub_->publish(
            make_float32(unit_cast<unit::time::s, unit::time::ms>(ave_solve_time)));
          perfomance_pub_->publish(
            make_float32(unit_cast<unit::time::s, unit::time::ms>(planner_->get_solve_time())));
        }
      }
    }
  }

  // Action Client
  void controler_send_target(Pose3d target_pose)
  {
    if (controller_running_) {
      RCLCPP_WARN(get_logger(), "Controller is already running");
      control_action_client_->async_cancel_all_goals();
    }
    end_ = false;
    using namespace std::placeholders;
    global_path_ = std::nullopt;
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header = make_header(MAP_FRAME, rclcpp::Clock().now());
    goal_msg.pose.pose = make_geometry_pose(target_pose);
    // 進捗状況を表示するFeedbackコールバックを設定
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.feedback_callback =
      [&](
        ClientGoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        RCLCPP_DEBUG(get_logger(), "Distance remaininf = %f", feedback->distance_remaining);
        controller_running_ = true;
      };
    send_goal_options.result_callback =
      [&](const ClientGoalHandleNavigateToPose::WrappedResult & result) {
        controller_running_ = false;
        end_ = true;
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Success!!!");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Goal was aborted");
            return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(get_logger(), "Goal was canceled");
            return;
          default:
            RCLCPP_ERROR(get_logger(), "Unknown result code");
            return;
        }
      };
    // controlerに送信
    control_action_client_->async_send_goal(goal_msg, send_goal_options);
  }
  void global_planner_send_target(Pose3d target_pose)
  {
    if (global_planner_running_) {
      RCLCPP_WARN(get_logger(), "Global Path Planner is already running");
      global_planning_action_client_->async_cancel_all_goals();
    }
    using namespace std::placeholders;
    global_path_ = std::nullopt;
    global_planner_running_ = true;
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header = make_header(MAP_FRAME, rclcpp::Clock().now());
    goal_msg.pose.pose = make_geometry_pose(target_pose);
    // 進捗状況を表示するFeedbackコールバックを設定
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    // global_plannerに送信
    send_goal_options.feedback_callback =
      [&](
        ClientGoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        global_planner_running_ = true;
      };
    send_goal_options.result_callback =
      [&](const ClientGoalHandleNavigateToPose::WrappedResult & result) {
        global_planner_running_ = false;
      };
    global_planning_action_client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  bool end_;
  bool planner_stop_;
  bool controller_running_;
  bool planner_running_;
  bool global_planner_running_;
  // param
  std::string MAP_FRAME;
  std::string ROBOT_FRAME;
  double PLANNING_PERIOD;
  double GLOBAL_PLANNING_PERIOD;
  mpc_config_t mpc_config_;
  double OBSTACLE_DETECT_DIST;
  double HALF_OBSTACLE_DETECT_DIST;
  size_t OBSTACLES_MAX_SIZE;
  double MAX_OBSTACLE_SIZE;
  double MIN_OBSTACLE_SIZE;
  double NEARBY_OBSTACLE_LIMIT;
  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;
  // action
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr control_action_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr global_planning_action_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr planning_action_client_;
  // subscriber
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;  // debug
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  // publisher
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr init_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr opti_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<extension_msgs::msg::TwistMultiArray>::SharedPtr opti_twists_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mpc_dt_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr perfomance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr perfomance_ave_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_pub_;
  // twist
  Twistd now_vel_;
  // pose
  std::optional<Pose3d> target_pose_;
  // planner
  std::optional<GridMap> map_;
  std::optional<Pathd> global_path_;
  std::shared_ptr<MPCPathPlanner> planner_;

  void publish_target(const Pose3d & base_link_pose)
  {
    static Timerd debug_timer;
    if (debug_timer.cyclic(1.0)) {
      geometry_msgs::msg::PoseStamped target_msg;
      target_msg.header = make_header(MAP_FRAME, rclcpp::Clock().now());
      target_msg.pose = make_geometry_pose(target_pose_.value());
      target_msg.pose.position.z = base_link_pose.position.z;
      target_pub_->publish(target_msg);
    }
  }

  struct obstacle_t
  {
    Vector2d pos;
    double dist;
    bool operator<(const obstacle_t & o) const { return o.dist < dist; };
  };
  std::priority_queue<obstacle_t> obstacles_;
  visualization_msgs::msg::MarkerArray obstacles_marker_;
  Vector3d obstacle_size_;

  void clear_obstacles()
  {
    std::priority_queue<obstacle_t> empty;
    obstacles_ = empty;  // 要素の削除
  }

  std::pair<casadi::DM, casadi::DM> set_obstacle(std_msgs::msg::ColorRGBA color)
  {
    using namespace casadi;
    using Sl = casadi::Slice;
    casadi::DM dm_obstacles = DM::zeros(2, OBSTACLES_MAX_SIZE);
    casadi::DM dm_obstacles_size = set_value(obstacle_size_.to_vector2());
    obstacle_t pre_obs;
    casadi::DM dm_obs = DM::zeros(2);
    for (size_t i = 0; i < OBSTACLES_MAX_SIZE; i++) {
      Vector2d obs_p = {EXECUSION_POINT_VALUE, EXECUSION_POINT_VALUE};
      if (!obstacles_.empty()) {
        obstacle_t obstacle = obstacles_.top();
        if (i != 0) {
          Vector2d diff = pre_obs.pos - obstacle.pos;
          // if (
          //   std::abs(diff.x) >= (NEARBY_OBSTACLE_LIMIT * obstacle_size_.x) ||
          //   std::abs(diff.y) >= (NEARBY_OBSTACLE_LIMIT * obstacle_size_.y))
          if (diff.norm() >= (NEARBY_OBSTACLE_LIMIT * obstacle_size_.z))
            obs_p = obstacle.pos;
          else {
            i--;
            obstacles_.pop();
            continue;
          }
        } else
          obs_p = obstacle.pos;
        pre_obs = obstacle;
        obstacles_.pop();
      }
      set_value(dm_obs, obs_p);
      dm_obstacles(Sl(), i) = dm_obs;
      obstacles_marker_.markers.at(i) = make_circle_maker(
        make_header(MAP_FRAME, rclcpp::Clock().now()), obs_p, i, color, obstacle_size_);
    }
    return std::make_pair(dm_obstacles, dm_obstacles_size);
  }

  void obstacles_detect(Pose3d robot_pose)
  {
    if (map_) {
      auto start = rclcpp::Clock().now();
      Vector2d robot = {robot_pose.position.x, robot_pose.position.y};
      clear_obstacles();
      // 探索範囲計算
      Vector2d start_cell = map_.value().get_grid_pos(
        robot.x - HALF_OBSTACLE_DETECT_DIST, robot.y - HALF_OBSTACLE_DETECT_DIST);
      Vector2d end_cell = map_.value().get_grid_pos(
        robot.x + HALF_OBSTACLE_DETECT_DIST, robot.y + HALF_OBSTACLE_DETECT_DIST);
      size_t start_x = static_cast<size_t>(start_cell.x);
      size_t start_y = static_cast<size_t>(start_cell.y);
      size_t end_x = static_cast<size_t>(end_cell.x);
      size_t end_y = static_cast<size_t>(end_cell.y);
      if (!(0 <= start_cell.x && start_cell.x < map_.value().info.width)) start_x = 0;
      if (!(0 <= start_cell.y && start_cell.y < map_.value().info.height)) start_y = 0;
      if (!(0 <= end_cell.x && end_cell.x < map_.value().info.width))
        end_x = map_.value().info.width;
      if (!(0 <= end_cell.y && end_cell.y < map_.value().info.height))
        end_y = map_.value().info.height;
#if defined(OBSTACLE_DETECT_DEBUG_OUTPUT)
      std::cout << "---------------------------------------------------" << std::endl;
      std::cout << "start cell:" << start_x << "," << start_y << std::endl;
      std::cout << "end cell:" << end_x << "," << end_y << std::endl;
#endif
      // 障害物探索
#pragma omp parallel for
      for (size_t y = start_y; y < end_y; y++) {
        for (size_t x = start_x; x < end_x; x++) {
          if (map_.value().is_wall(x, y)) {
            Vector2d obstacle = map_.value().grid_to_pos(x, y);
            double dist = (obstacle - robot).norm();
#pragma omp critical
            obstacles_.push({obstacle, dist});
          }
        }
      }
      // 障害物サイズ計算
      double vel = now_vel_.linear.norm();
      if (vel > mpc_config_.max_vel) vel = mpc_config_.max_vel;
      double obstacle_size = transform_range<double>(
        vel, 0.0, mpc_config_.max_vel, MIN_OBSTACLE_SIZE, MAX_OBSTACLE_SIZE);
      double theta = robot_pose.orientation.get_rpy().z;
      double x_size =
        MIN_OBSTACLE_SIZE + (obstacle_size - MIN_OBSTACLE_SIZE) * std::abs(cos(theta));
      double y_size =
        MIN_OBSTACLE_SIZE + (obstacle_size - MIN_OBSTACLE_SIZE) * std::abs(sin(theta));
      obstacle_size_ = {x_size, y_size, obstacle_size};
#if defined(OBSTACLE_DETECT_DEBUG_OUTPUT)
      std::cout << "---------------------------------------------------" << std::endl;
      std::cout << "vel:" << vel << std::endl;
      std::cout << "obstacle_size:" << obstacle_size_ << std::endl;
#endif

#if defined(OBSTACLE_DETECT_DEBUG_OUTPUT)
      std::cout << "obstacles size:" << obstacles_.size() << std::endl;
      std::cout << "obstacles detect time:" << (rclcpp::Clock().now() - start).seconds()
                << std::endl;
#endif
    }
  }

  // MPU追加制約
  casadi::MX mx_obstacles_, mx_obstacles_size_;
  void init_parameter(casadi::Opti & opti)
  {
    mx_obstacles_ = opti.parameter(2, OBSTACLES_MAX_SIZE);  // 障害物パラメータ
    mx_obstacles_size_ = opti.parameter(2);
  }

  void add_cost_function(casadi::MX & cost, const casadi::MX & X, const casadi::MX & U) {}

  void add_constraints(casadi::Opti & opti, const casadi::MX & X, const casadi::MX & U)
  {
    using namespace casadi;
    using Sl = casadi::Slice;
    for (size_t i = 1; i < (size_t)X.size2(); i++) {
      for (size_t j = 0; j < OBSTACLES_MAX_SIZE; j++) {
        opti.subject_to(get_guard_circle_subject(
          X(Sl(3, 5), i), mx_obstacles_(Sl(0, 2), j),
          MX::vertcat({mx_obstacles_size_(Sl(0)), mx_obstacles_size_(Sl(1))}),
          "keep out"));  // 障害物制約
      }
    }
  }

  void set_user_param(casadi::Opti & opti)
  {
    auto dm_obstacles = set_obstacle(make_color(1.0, 0.0, 0.0, 0.1));
    opti.set_value(mx_obstacles_, dm_obstacles.first);        // 障害物の位置追加
    opti.set_value(mx_obstacles_size_, dm_obstacles.second);  // 障害物のsize追加
    obstacles_pub_->publish(obstacles_marker_);
#if defined(OBSTACLE_DETECT_DEBUG_OUTPUT)
    std::cout << "dm_obstacles_size:" << dm_obstacles_size << std::endl;
    std::cout << "set obstacles param" << std::endl;
#endif
  }
};