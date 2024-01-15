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
#include "common_lib/planner/mpc_path_planner.hpp"

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
class MPCPathPlanning : public ExtensionNode
{
public:
  MPCPathPlanning(const rclcpp::NodeOptions &options) : MPCPathPlanning("", options) {}
  MPCPathPlanning(
      const std::string &name_space = "",
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : ExtensionNode("mpc_path_planning_node", name_space, options),
        broadcaster_(this),
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
    std::string CMD_VEL_TOPIC =
        param<std::string>("mpc_path_planning.topic_name.cmd_vel", "/cmd_vel");
    // frame
    MAP_FRAME = param<std::string>("mpc_path_planning.tf_frame.map_frame", "map");
    ODOM_FRAME = param<std::string>("mpc_path_planning.tf_frame.odom_frame", "odom");
    ROBOT_FRAME = param<std::string>("mpc_path_planning.tf_frame.robot_frame", "base_link");
    // setup
    CONTROL_PERIOD = param<double>("mpc_path_planning.control_period", 0.001);
    // mpc
    mpc_config_t mpc_config;
    mpc_config.dt = param<double>("mpc_path_planning.mpc.dt", 0.06);
    double HORIZON_TIME = param<double>("mpc_path_planning.mpc.horizon_time", 7.0);
    mpc_config.horizon = static_cast<size_t>(HORIZON_TIME / mpc_config.dt);
    mpc_config.max_vel = param<double>("mpc_path_planning.mpc.max_vel", 1.0);                 // [m/s]
    mpc_config.max_angular = param<double>("mpc_path_planning.mpc.max_angular", 1.0);         // [m/s]
    mpc_config.max_acc = param<double>("mpc_path_planning.mpc.max_acc", 0.98);                // [m/s^2]
    mpc_config.max_angular_acc = param<double>("mpc_path_planning.mpc.max_angular_acc", 1.0); // [rad/s^2]
    std::vector<double> STATE_WEIGHT = param<std::vector<double>>("mpc_path_planning.mpc.weight.state", std::vector<double>{10, 10, 6, 200, 200, 60});
    std::vector<double> FINAL_STATE_WEIGHT = param<std::vector<double>>("mpc_path_planning.mpc.weight.final_state", std::vector<double>{10, 10, 6, 200, 200, 60});
    std::vector<double> CONTROL_WEIGHT = param<std::vector<double>>("mpc_path_planning.mpc.weight.control", std::vector<double>{3, 3, 2});
    mpc_config.state_weight << STATE_WEIGHT[0], STATE_WEIGHT[1], STATE_WEIGHT[2], STATE_WEIGHT[3], STATE_WEIGHT[4], STATE_WEIGHT[5];
    mpc_config.final_state_weight << FINAL_STATE_WEIGHT[0], FINAL_STATE_WEIGHT[1], FINAL_STATE_WEIGHT[2], FINAL_STATE_WEIGHT[3], FINAL_STATE_WEIGHT[4], FINAL_STATE_WEIGHT[5];
    mpc_config.control_weight << CONTROL_WEIGHT[0], CONTROL_WEIGHT[1], CONTROL_WEIGHT[2];
    mpc_config.lpf_xy_gain = mpc_config.dt / (mpc_config.dt + mpc_config.xy_vel_time_constant);
    mpc_config.lpf_theta_gain = mpc_config.dt / (mpc_config.dt + mpc_config.theta_vel_time_constant);
    std::string IPOPT_SB = param<std::string>("mpc_path_planning.mpc.ipopt.sb", "yes");
    std::string IPOPT_LINEAR_SOLVER = param<std::string>("mpc_path_planning.mpc.ipopt.linear_solver", "numps"); // numpsは遅い ma27はHSLライブラリ必要
    int IPOPT_MAX_ITER = param<int>("mpc_path_planning.mpc.ipopt.max_iter", 500);
    double IPOPT_ACCEPTABLE_TOL = param<double>("mpc_path_planning.mpc.ipopt.acceptable_tol", 0.000001);
    double IPOPT_COMPL_INF_TOL = param<double>("mpc_path_planning.mpc.ipopt.compl_inf_tol", 0.0001);
    mpc_config.solver_option =
        {
            {"ipopt.sb", IPOPT_SB.c_str()}, // コンソールにヘッダを出力しない
            // {"ipopt.linear_solver", "ma27"}, // numpsは遅い ma27はHSLライブラリ必要
            {"ipopt.linear_solver", IPOPT_LINEAR_SOLVER.c_str()}, // numpsは遅い ma27はHSLライブラリ必要
            {"ipopt.max_iter", IPOPT_MAX_ITER},
            {"ipopt.acceptable_tol", IPOPT_ACCEPTABLE_TOL},
            {"ipopt.compl_inf_tol", IPOPT_COMPL_INF_TOL},
            //{"ipopt.tol", 1.0e-8},
            {"ipopt.print_level", 0},
            {"print_time", false},
            {"ipopt.warm_start_init_point", "yes"},
            //{"ipopt.hessian_approximation", "limited-memory"},//使えてない
            //{"ipopt.fixed_variable_treatment","make_constraint"},
            {"expand", true}};
    std::string GRID_PATH_PLANNING = param<std::string>("mpc_path_planning.mpc.grid_path_planner", "a_star");
    // init
    RCLCPP_INFO(this->get_logger(), "Initialization !");
    initialization_ = true;
    planner_ = std::make_shared<MPCPathPlanner>(mpc_config);
    planner_->timer = [&]()
    { return now().seconds(); };
    if (GRID_PATH_PLANNING.compare("wave_propagation") == 0)
      planner_->set_grid_path_planner(std::make_shared<WavePropagation>());
    else if (GRID_PATH_PLANNING.compare("a_star") == 0)
      planner_->set_grid_path_planner(std::make_shared<AStar>());
    else
      RCLCPP_WARN(this->get_logger(), "grid path planning not used");
    RCLCPP_INFO(this->get_logger(), "grid path planning: %s", GRID_PATH_PLANNING.c_str());
    planner_->set_kinematics_model(omni_directional_model(mpc_config));
    planner_->init_solver();
    // publisher
    grid_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(GRIDPATH_TOPIC, rclcpp::QoS(10));
    opti_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(OPTIPATH_TOPIC, rclcpp::QoS(10));
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(CMD_VEL_TOPIC, rclcpp::QoS(10));
    // subscriber
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        MAP_TOPIC, rclcpp::QoS(10).reliable(),
        [&](const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
        { map_msg_ = msg; });
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        TARGET_TOPIC, rclcpp::QoS(10),
        [&](geometry_msgs::msg::PoseStamped::SharedPtr msg)
        { target_pose_ = make_pose(msg->pose); });
    // timer
    main_timer_ = this->create_wall_timer(1s * CONTROL_PERIOD, [&]()
                                          {
      if (!tf_buffer_.canTransform(
            MAP_FRAME, ROBOT_FRAME, rclcpp::Time(0),
            tf2::durationFromSec(1.0))) {  // 変換無いよ
        RCLCPP_WARN(
          this->get_logger(), "%s %s can not Transform", MAP_FRAME.c_str(), ROBOT_FRAME.c_str());
        return;
      }
      auto map_to_base_link = lookup_transform(tf_buffer_, MAP_FRAME, ROBOT_FRAME);
      if (map_to_base_link) {
        base_link_pose_ = make_pose(map_to_base_link.value().transform);
        RCLCPP_INFO_CHANGE(0, this->get_logger(), "get base_link pose");
        if (map_msg_) {
          GridMap map = make_gridmap(*map_msg_);
          planner_->set_map(map);
          RCLCPP_INFO_CHANGE(1, this->get_logger(), "get map");
          if (target_pose_) {
            std::cout << base_link_pose_ << std::endl;
            std::cout << map.info.resolution << std::endl;
            PathPointd start, end;
            start.pose = base_link_pose_;
            end.pose = target_pose_.value();
            // path planning
            Pathd opti_path = planner_->pathplanning(start, end);
            Pathd grid_path = planner_->get_grid_path();
            // publish path
            auto [g_ppath, g_vpath, g_apath] =
              make_nav_path(make_header(MAP_FRAME, rclcpp::Clock().now()), grid_path);
            grid_path_pub_->publish(g_ppath);
            if(planner_->optimization()) { //最適化成功
              auto [o_ppath, o_vpath, o_apath] =
                make_nav_path(make_header(MAP_FRAME, rclcpp::Clock().now()), opti_path);
              opti_path_pub_->publish(o_ppath);
              // publish twist
              // auto now_time = this->get_clock()->now();
              // auto duration = now_time - latest_sub_time_; //latest_sub_msg_->header.stamp;
              // double control_time = duration.seconds();
              // if(control_time < 0)
              //     control_time = 0;
              // double dt = 0.135;
              // size_t horizon = std::round(control_time/dt);
              // geometry_msgs::msg::Twist twist;
              // twist.linear.x = 0;
              // twist.linear.y = 0;
              // twist.angular.z = 0;
              // if(horizon < latest_sub_msg_->twists.size())
              // {
              //     auto &target_twist0 = latest_sub_msg_->twists[horizon];
              //     auto nhorizon = horizon;
              //     if(horizon+1 < latest_sub_msg_->twists.size())
              //         nhorizon += 1;
              //     auto &target_twist1 = latest_sub_msg_->twists[nhorizon];
              //     Transformd v0(target_twist0.x, target_twist0.y, target_twist0.theta);
              //     Transformd v1(target_twist1.x, target_twist1.y, target_twist1.theta);
              //     double t = (control_time - dt*horizon) / dt;
              //     if(t < 0)
              //         t = 0;
              //     // RCLCPP_INFO(this->get_logger(), "hori: %d, %.3f", horizon, t);
              //     auto v = v0 + (v1 - v0)*t;
              //     // tfから取ったほうがいい？
              //     double t_angle = target_twist0.pose_theta + (target_twist1.pose_theta-target_twist0.pose_theta)*t;
              //     auto v_xy = v.make_vector2();
              //     v_xy.rotate(-get_yaw(base_link_tf.getRotation()));
              //   twist.linear.x = v_xy.x;
              //   twist.linear.y = v_xy.y;
              //   twist.angular.z = v.theta;
              // }
              // else
              // {
              //   // RCLCPP_INFO(this->get_logger(), "error");
              //   twist.linear.x = 0;
              //   twist.linear.y = 0;
              //   twist.angular.z = 0;
              //   is_data = false;
              // }
              // cmd_vel_pub_->publish(twist);
            }
            target_pose_ = std::nullopt;
          }
        }
      } });
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
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr opti_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  // pose
  Pose3d base_link_pose_;
  std::optional<Pose3d> target_pose_;
  // map
  nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
  // planner
  std::shared_ptr<MPCPathPlanner> planner_;

  Vector2d vector2_cast(const PathPointd &p) { return {p.pose.position.x, p.pose.position.y}; }
  PathPointd pathpoint_cast(const Vector2d &v)
  {
    PathPointd p;
    p.pose.position.x = v.x;
    p.pose.position.y = v.y;
    return p;
  }
};