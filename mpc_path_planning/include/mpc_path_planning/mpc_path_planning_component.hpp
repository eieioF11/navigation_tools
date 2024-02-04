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
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <std_msgs/msg/empty.hpp>

#include "common_lib/ros2_utility/extension_msgs_util.hpp"
#include "common_lib/ros2_utility/marker_util.hpp"
#include "common_lib/ros2_utility/msg_util.hpp"
#include "common_lib/ros2_utility/ros_opencv_util.hpp"
#include "common_lib/ros2_utility/tf_util.hpp"
#include "extension_node/extension_node.hpp"
// other
#include "common_lib/common_lib.hpp"

#define PLANNER_DEBUG_OUTPUT
#define USE_OMP
// grid path planner
#include "common_lib/planner/a_star.hpp"
#include "common_lib/planner/dijkstra.hpp"
#include "common_lib/planner/extension_a_star.hpp"
#include "common_lib/planner/wave_propagation.hpp"
// mpc
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
#define OBSTACLE_DETECT_DEBUG_OUTPUT
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
          {"ipopt.fixed_variable_treatment",
           "make_constraint"}, // 固定変数をどのように処理するか  make_constraint:変数を固定する等価制約を追加
          {"expand", true},
      };
    };
    mpc_config_.solver_option = solver_option();
    // mpc_config_.solver_option = MPCPathPlanner::default_solver_option();//default 設定
    OBSTACLE_DETECT_DIST = param<double>("mpc_path_planning.obstacle_detect.dist", 5.0);
    OBSTACLE_SIZE = param<double>("mpc_path_planning.obstacle_detect.obstacle_size", 0.005);
    OBSTACLES_MAX_SIZE = static_cast<size_t>(param<int>("mpc_path_planning.obstacle_detect.list_size", 5));
    NEARBY_OBSTACLE_LIMIT = param<double>("mpc_path_planning.obstacle_detect.nearby_obstacle_limit", 0.8);
    // init
    RCLCPP_INFO(this->get_logger(), "Initialization !");
    HALF_OBSTACLE_DETECT_DIST = OBSTACLE_DETECT_DIST / 2.0;
    target_pose_ = std::nullopt;
    vel_error_.linear = {0.0, 0.0, 0.0};
    vel_error_.angular = {0.0, 0.0, 0.0};
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
    opti_path_pub_ =
        this->create_publisher<nav_msgs::msg::Path>(OPTIPATH_TOPIC, rclcpp::QoS(10).reliable());
    opti_twists_pub_ = this->create_publisher<extension_msgs::msg::TwistMultiArray>(
        OPTITWISTS_TOPIC, rclcpp::QoS(10).reliable());
    perfomance_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "mpc_path_planning/solve_time", rclcpp::QoS(5));
    obstacles_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "mpc_path_planning/obstacles", rclcpp::QoS(5));
    // subscriber
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        TARGET_TOPIC, rclcpp::QoS(10), [&](geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
        target_pose_ = make_pose(msg->pose);
        global_path_ = std::nullopt; });
    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        GLOBALPATH_TOPIC, rclcpp::QoS(10).reliable(), [&](const nav_msgs::msg::Path::SharedPtr msg)
        {
        RCLCPP_INFO(this->get_logger(), "get global_path!");
        global_path_ = make_path(*msg); });
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        ODOM_TOPIC, rclcpp::QoS(10),
        [&](nav_msgs::msg::Odometry::SharedPtr msg)
        { now_vel_ = make_twist(*msg); });
    end_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "mpc_path_planning/end", rclcpp::QoS(10).reliable(),
        [&](std_msgs::msg::Empty::SharedPtr msg)
        {
          RCLCPP_INFO(this->get_logger(), "end!");
          target_pose_ = std::nullopt;
          global_path_ = std::nullopt;
        });
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        MAP_TOPIC, rclcpp::QoS(10).reliable(),
        [&](const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
        { map_ = make_gridmap(*msg); });
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::QoS(10), [&](const geometry_msgs::msg::Twist::SharedPtr msg)
        { log(
              now_vel_.linear.x, now_vel_.linear.y, now_vel_.angular.z, msg->linear.x, msg->linear.y,
              msg->angular.z); });
    // timer
    path_planning_timer_ = this->create_wall_timer(1s * PLANNING_PERIOD, [&]()
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
        if (global_path_) {
#if defined(NON_HOLONOMIC)
          now_vel_.linear.y = 0.0;
#endif
          planner_->set_grid_path(global_path_.value());
          if (target_pose_) {
#if defined(PLANNING_DEBUG_OUTPUT)
            std::cout << "----------------------------------------------------------" << std::endl;
            std::cout << "now_vel:" << now_vel_ << std::endl;
            std::cout << "vel_error_:" << vel_error_ << std::endl;
            std::cout << "end:" << target_pose_.value() << std::endl;
            std::cout << "start:" << base_link_pose << std::endl;
            std::cout << "end:" << target_pose_.value() << std::endl;
#endif
            PathPointd start = make_pathpoint<double>(base_link_pose, now_vel_);
            PathPointd end = make_pathpoint<double>(target_pose_.value());
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
              perfomance_pub_->publish(
                make_float32(unit_cast<unit::time::s, unit::time::ms>(planner_->get_solve_time())));
            }
          }
        }
      } });
    init_data_logger({"odom_vx", "odom_vy", "odom_w", "u_vx", "u_vy", "u_w"});
  }

private:
  // param
  std::string MAP_FRAME;
  std::string ROBOT_FRAME;
  double PLANNING_PERIOD;
  mpc_config_t mpc_config_;
  double OBSTACLE_DETECT_DIST;
  double HALF_OBSTACLE_DETECT_DIST;
  size_t OBSTACLES_MAX_SIZE;
  double OBSTACLE_SIZE;
  double NEARBY_OBSTACLE_LIMIT;
  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;
  // timer
  rclcpp::TimerBase::SharedPtr path_planning_timer_;
  // subscriber
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_; // debug
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr end_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  // publisher
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr init_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr opti_path_pub_;
  rclcpp::Publisher<extension_msgs::msg::TwistMultiArray>::SharedPtr opti_twists_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr perfomance_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_pub_;
  // twist
  Twistd now_vel_;
  Twistd vel_error_;
  // pose
  std::optional<Pose3d> target_pose_;
  // planner
  std::optional<GridMap> map_;
  std::optional<Pathd> global_path_;
  std::shared_ptr<MPCPathPlanner> planner_;

  struct obstacle_t
  {
    Vector2d pos;
    double dist;
    bool operator<(const obstacle_t &o) const { return o.dist < dist; };
  };
  std::priority_queue<obstacle_t> obstacles_;
  visualization_msgs::msg::MarkerArray obstacles_marker_;

  void clear_obstacles()
  {
    std::priority_queue<obstacle_t> empty;
    obstacles_ = empty; // 要素の削除
  }

  void obstacles_detect(Pose3d robot_pose)
  {
    if (map_)
    {
      auto start = rclcpp::Clock().now();
      Vector2d robot = {robot_pose.position.x, robot_pose.position.y};
      clear_obstacles();
      Vector2d start_cell = map_.value().get_grid_pos(robot.x - HALF_OBSTACLE_DETECT_DIST, robot.y - HALF_OBSTACLE_DETECT_DIST);
      Vector2d end_cell = map_.value().get_grid_pos(robot.x + HALF_OBSTACLE_DETECT_DIST, robot.y + HALF_OBSTACLE_DETECT_DIST);
      size_t start_x = static_cast<size_t>(start_cell.x);
      size_t start_y = static_cast<size_t>(start_cell.y);
      size_t end_x = static_cast<size_t>(end_cell.x);
      size_t end_y = static_cast<size_t>(end_cell.y);
      if (!(0 <= start_x && start_x < map_.value().info.width))
        start_x = 0;
      if (!(0 <= start_y && start_y < map_.value().info.height))
        start_y = 0;
      if (!(0 <= end_x && end_x < map_.value().info.width))
        end_x = map_.value().info.width;
      if (!(0 <= end_y && end_y < map_.value().info.height))
        end_y = map_.value().info.height;
#if defined(OBSTACLE_DETECT_DEBUG_OUTPUT)
      std::cout << "---------------------------------------------------" << std::endl;
      std::cout << "start cell:" << start_x << "," << start_y << std::endl;
      std::cout << "end cell:" << end_x << "," << end_y << std::endl;
#endif
#pragma omp parallel for
      for (size_t y = start_y; y < end_y; y++)
      {
        for (size_t x = start_x; x < end_x; x++)
        {
          if (map_.value().is_wall(x, y))
          {
            Vector2d obstacle = map_.value().grid_to_pos(x, y);
            double dist = (obstacle - robot).norm();
#pragma omp critical
            obstacles_.push({obstacle, dist});
          }
        }
      }
      if (!target_pose_)
      {
        obstacle_t pre_obs;
        for (size_t i = 0; i < OBSTACLES_MAX_SIZE; i++)
        {
          Vector2d obs_p = {EXECUSION_POINT_VALUE, EXECUSION_POINT_VALUE};
          if (!obstacles_.empty())
          {
            if (i != 0)
            {
              if ((pre_obs.pos - obstacles_.top().pos).norm() >= (NEARBY_OBSTACLE_LIMIT * OBSTACLE_SIZE))
                obs_p = obstacles_.top().pos;
              else
              {
                i--;
                obstacles_.pop();
                continue;
              }
            }
            else
              obs_p = obstacles_.top().pos;
            pre_obs = obstacles_.top();
            obstacles_.pop();
          }
          obstacles_marker_.markers.at(i) = make_point_maker(
              make_header(MAP_FRAME, rclcpp::Clock().now()), obs_p, i, make_color(1.0, 1.0, 0.0, 0.1),
              OBSTACLE_SIZE);
        }
        obstacles_pub_->publish(obstacles_marker_);
      }
#if defined(OBSTACLE_DETECT_DEBUG_OUTPUT)
      std::cout << "obstacles size:" << obstacles_.size() << std::endl;
      std::cout << "obstacles detect time:" << (rclcpp::Clock().now() - start).seconds()
                << std::endl;
#endif
    }
  }

  // MPU追加制約
  casadi::MX mx_obstacles_;
  void init_parameter(casadi::Opti &opti)
  {
    mx_obstacles_ = opti.parameter(2, OBSTACLES_MAX_SIZE); // 障害物パラメータ
  }

  void add_cost_function(casadi::MX &cost, const casadi::MX &X, const casadi::MX &U) {}

  void add_constraints(casadi::Opti &opti, const casadi::MX &X, const casadi::MX &U)
  {
    using namespace casadi;
    using Sl = casadi::Slice;
    for (size_t i = 1; i < (size_t)X.size2(); i++)
    {
      for (size_t j = 0; j < OBSTACLES_MAX_SIZE; j++)
        opti.subject_to(get_guard_circle_subject(
            X(Sl(3, 5), i), mx_obstacles_(Sl(), j), MX::vertcat({OBSTACLE_SIZE, OBSTACLE_SIZE}),
            "keep out")); // 障害物制約
    }
  }

  void set_user_param(casadi::Opti &opti)
  {
    using namespace casadi;
    using Sl = casadi::Slice;
    casadi::DM dm_obstacles = DM::zeros(2, OBSTACLES_MAX_SIZE);
    obstacle_t pre_obs;
    for (size_t i = 0; i < OBSTACLES_MAX_SIZE; i++)
    {
      Eigen::Vector2d obs_p_mat =
          make_eigen_vector2(Vector2d{EXECUSION_POINT_VALUE, EXECUSION_POINT_VALUE});
      casadi::DM dm_obs = DM::zeros(2);
      if (!obstacles_.empty())
      {
        if (i != 0)
        {
          if ((pre_obs.pos - obstacles_.top().pos).norm() >= (NEARBY_OBSTACLE_LIMIT * OBSTACLE_SIZE))
            obs_p_mat << obstacles_.top().pos.x, obstacles_.top().pos.y;
          else
          {
            i--;
            obstacles_.pop();
            continue;
          }
        }
        else
          obs_p_mat << obstacles_.top().pos.x, obstacles_.top().pos.y;
        pre_obs = obstacles_.top();
        obstacles_.pop();
      }
      std::copy(obs_p_mat.data(), obs_p_mat.data() + obs_p_mat.size(), dm_obs.ptr());
      dm_obstacles(Sl(), i) = dm_obs;
      obstacles_marker_.markers.at(i) = make_point_maker(
          make_header(MAP_FRAME, rclcpp::Clock().now()), Vector2d{obs_p_mat(0), obs_p_mat(1)}, i,
          make_color(1.0, 0.0, 0.0, 0.1), OBSTACLE_SIZE);
    }
    opti.set_value(mx_obstacles_, dm_obstacles); // 障害物の位置追加
    obstacles_pub_->publish(obstacles_marker_);
#if defined(OBSTACLE_DETECT_DEBUG_OUTPUT)
    std::cout << "set obstacles param" << std::endl;
#endif
  }
};