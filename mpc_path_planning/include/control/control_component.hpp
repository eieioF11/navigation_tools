#pragma once
#include "mpc_path_planning/mpc_path_planning_component.hpp"

using namespace common_lib;
using namespace std::chrono_literals;
class Control : public ExtensionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  Control(const rclcpp::NodeOptions & options) : Control("", options) {}
  Control(
    const std::string & name_space = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : ExtensionNode("control_node", name_space, options),
    tf_buffer_(get_clock()),
    listener_(tf_buffer_)
  {
    RCLCPP_INFO(get_logger(), "start control_node");
    // get param
    std::string OPTIPATH_TOPIC =
      param<std::string>("control.topic_name.opti_path", "mpc_path_planning/opti_path");
    std::string OPTITWISTS_TOPIC =
      param<std::string>("control.topic_name.opti_twists", "mpc_path_planning/twists");
    std::string CMD_VEL_TOPIC = param<std::string>("control.topic_name.cmd_vel", "/cmd_vel");
    std::string ODOM_TOPIC = param<std::string>("control.topic_name.odom", "/odom");
    // frame
    MAP_FRAME = param<std::string>("control.tf_frame.map_frame", "map");
    ROBOT_FRAME = param<std::string>("control.tf_frame.robot_frame", "base_link");
    // setup
    CONTROL_PERIOD = param<double>("control.control_period", 0.001);
    TIME_OUT = param<double>("control.time_out", 10.0);
    time_sync_ = param<bool>("control.time_sync", false);
    // 収束判定
    GOAL_POS_RANGE = param<double>("control.goal.pos_range", 0.01);
    GOAL_ANGLE_RANGE = unit_cast<unit::angle::rad>(param<double>("control.goal.angle_range", 0.1));
    GOAL_MIN_VEL_RANGE = param<double>("control.goal.min_vel_range", 0.001);
    GOAL_MIN_ANGULAR_RANGE = param<double>("control.goal.min_angular_range", 0.001);
    // init
    RCLCPP_INFO(get_logger(), "Initialization !");
    opti_twists_ = std::nullopt;
    opti_path_ = std::nullopt;
    MPC_DT = 0.1;
    inv_MPC_DT = 1.0 / MPC_DT;
    pre_control_time_ = get_clock()->now();
    // publisher
    cmd_vel_pub_ =
      create_publisher<geometry_msgs::msg::Twist>(CMD_VEL_TOPIC, rclcpp::QoS(10));
    linear_vel_pub_ = create_publisher<std_msgs::msg::Float32>(
      "mpc_path_planning/linear_vel", rclcpp::QoS(5));
    angular_vel_pub_ = create_publisher<std_msgs::msg::Float32>(
      "mpc_path_planning/angular_vel", rclcpp::QoS(5));
    perfomance_pub_ = create_publisher<std_msgs::msg::Float32>(
      "mpc_path_planning/control_period", rclcpp::QoS(5));
    control_time_pub_ = create_publisher<std_msgs::msg::Float32>(
      "mpc_path_planning/control_time", rclcpp::QoS(5));
    cmd_vel_pub_->publish(stop());
    opti_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      OPTIPATH_TOPIC, rclcpp::QoS(10).reliable(),
      [&](const nav_msgs::msg::Path::SharedPtr msg) { opti_path_ = *msg; });
    opti_twists_sub_ = create_subscription<extension_msgs::msg::TwistMultiArray>(
      OPTITWISTS_TOPIC, rclcpp::QoS(10),
      [&](extension_msgs::msg::TwistMultiArray::SharedPtr msg) {
        opti_twists_ = *msg;
        // opti_twists_.value().header.stamp = get_clock()->now();
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      ODOM_TOPIC, rclcpp::QoS(10),
      [&](nav_msgs::msg::Odometry::SharedPtr msg) { odom_vel_ = make_twist(*msg); });
    mpc_dt_sub_ = create_subscription<std_msgs::msg::Float32>(
      "mpc_path_planning/dt", rclcpp::QoS(10).reliable(),
      [&](std_msgs::msg::Float32::SharedPtr msg) {
        MPC_DT = msg->data;
        if (approx_zero(MPC_DT)) {
          RCLCPP_ERROR(get_logger(), "MPC_DT is 0.0");
          return;
        }
        inv_MPC_DT = 1.0 / MPC_DT;
      });
    // action
    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<NavigateToPose>(
      this, "mpc_path_planning/control", std::bind(&Control::handle_goal, this, _1, _2),
      std::bind(&Control::handle_cancel, this, _1), std::bind(&Control::handle_accepted, this, _1));
    init_data_logger(
      {"u_vx", "u_vy", "u_w", "odom_vx", "odom_vy", "odom_w", "t_x", "t_y", "t_theta", "x", "y",
       "theta"});
    if(time_sync_)
    {
      clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("mpc_path_planning/clock", rclcpp::QoS(10));
      timer_ = create_wall_timer(1ns, [&]() {
        rosgraph_msgs::msg::Clock clock_msg;
        clock_msg.clock = get_clock()->now();
        clock_pub_->publish(clock_msg);
      });
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const NavigateToPose::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Controler Received goal request");
    target_pose_ = make_pose(goal->pose.pose);
    std::cout << "goal:" << target_pose_ << std::endl;
    control_time_over_ = false;
    cmd_vel_pub_->publish(stop());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Controler Received request to cancel goal");
    (void)goal_handle;
    opti_twists_ = std::nullopt;
    opti_path_ = std::nullopt;
    cmd_vel_pub_->publish(stop());
    controller_running_ = false;
    control_time_over_ = false;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&Control::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    Timerd time_out_timer;
    std::chrono::nanoseconds control_period(
      static_cast<int>(unit_cast<unit::time::s, unit::time::ns>(CONTROL_PERIOD)));
    rclcpp::Rate loop_rate(control_period);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    auto result = std::make_shared<NavigateToPose::Result>();
    controller_running_ = true;
    while (rclcpp::ok() && controller_running_) {
      if (control(feedback)) {
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Controler Goal succeeded");
        return;
      }
      if (!control_time_over_) time_out_timer.start();
      if (time_out_timer.elapsed() > TIME_OUT) {
        RCLCPP_INFO(get_logger(), "Controler Time out");
        return;
      }
      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }
    cmd_vel_pub_->publish(stop());
  }

  bool control(const NavigateToPose::Feedback::SharedPtr feedback)
  {
    if (!tf_buffer_.canTransform(
          ROBOT_FRAME, MAP_FRAME, rclcpp::Time(0),
          tf2::durationFromSec(1.0))) {  // 変換無いよ
      RCLCPP_WARN(
        get_logger(), "%s %s can not Transform", MAP_FRAME.c_str(), ROBOT_FRAME.c_str());
      return true;
    }
    auto map_to_base_link = lookup_transform(tf_buffer_, ROBOT_FRAME, MAP_FRAME);
    auto now_time = get_clock()->now();
    double vel = 0.;
    double angular = 0.;
    if (map_to_base_link && opti_path_ && opti_twists_) {
      Pathd opti_path = make_path(opti_path_.value(), opti_twists_.value());
      Pose3d base_link_pose = make_pose(map_to_base_link.value().transform);
      RCLCPP_INFO_CHANGE(0, get_logger(), "get opti_path");
#if defined(CONTROL_DEBUG_OUTPUT)
      std::cout << "----------------------------------------------------------" << std::endl;
      std::cout << "mpc_dt:" << MPC_DT << std::endl;
#endif
      // 制御出力
      Twistd cmd_vel = make_twist(stop());
      auto duration = now_time - opti_twists_.value().header.stamp;
      double control_time = duration.seconds();
      if (control_time < 0) control_time = 0;
      size_t n0 = std::round(control_time * inv_MPC_DT);
      control_time_pub_->publish(
        make_float32(unit_cast<unit::time::s, unit::time::ms>(control_time)));
      if (n0 < opti_path.points.size()) {
        auto & target_twist0 = opti_path.points[n0].velocity;
        auto n1 = n0;
        if (n0 + 1 < opti_path.points.size()) n1 += 1;
        auto & target_twist1 = opti_path.points[n1].velocity;
#if defined(CONTROL_DEBUG_OUTPUT)
        std::cout << "n0:" << n0 << " n1:" << n1 << std::endl;
        std::cout << "control_time:" << control_time << std::endl;
        std::cout << "target_twist0:" << target_twist0 << std::endl;
        std::cout << "target_twist1:" << target_twist1 << std::endl;
#endif
        Vector3d v0 = {target_twist0.linear.x, target_twist0.linear.y, target_twist0.angular.z};
        Vector3d v1 = {target_twist1.linear.x, target_twist1.linear.y, target_twist1.angular.z};
        double t = (control_time - MPC_DT * n0);
        if (t < 0) t = 0;
        Vector3d v = v0 + ((v1 - v0) * inv_MPC_DT) * t;  // 線形補間
#if defined(CONTROL_DEBUG_OUTPUT)
        std::cout << "t:" << t << std::endl;
        std::cout << "v:" << v << std::endl;
#endif
        Vector2d v_xy = {v.x, v.y};
#if !defined(NON_HOLONOMIC)
        v_xy.rotate(-base_link_pose.orientation.get_rpy().z);
#if defined(CONTROL_DEBUG_OUTPUT)
        std::cout << "rot_v_xy:" << v_xy << std::endl;
#endif
#endif
        cmd_vel.linear.x = v_xy.x;
        cmd_vel.linear.y = v_xy.y;
        cmd_vel.angular.z = v.z;
        control_time_over_ = false;
      } else {
        RCLCPP_WARN(get_logger(), "control_time over opti_path.points.size()");
        control_time_over_ = true;
      }
      target_pose_.position.z = base_link_pose.position.z = 0.0;
      double target_dist = Vector3d::distance(target_pose_.position, base_link_pose.position);
      double target_diff_angle =
        std::abs(target_pose_.orientation.get_rpy().z - base_link_pose.orientation.get_rpy().z);
      vel = cmd_vel.linear.norm();
      angular = cmd_vel.angular.norm();
      log(
        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z, odom_vel_.linear.x,
        odom_vel_.linear.y, odom_vel_.angular.z, target_pose_.position.x, target_pose_.position.y,
        target_pose_.orientation.get_rpy().z, base_link_pose.position.x, base_link_pose.position.y,
        base_link_pose.orientation.get_rpy().z);
#if defined(CONTROL_DEBUG_OUTPUT)
      std::cout << "target_dist:" << target_dist << std::endl;
      std::cout << "target_diff_angle:" << target_diff_angle << std::endl;
      std::cout << "vel:" << vel << std::endl;
      std::cout << "angular:" << angular << std::endl;
#endif
      // feedback
      feedback->current_pose.pose = make_geometry_pose(base_link_pose);
      feedback->distance_remaining = target_dist;
      // ゴール判定
      if (target_dist < GOAL_POS_RANGE) {
        if (target_diff_angle < GOAL_ANGLE_RANGE) {
          if (
            approx_zero(vel, GOAL_MIN_VEL_RANGE) && approx_zero(angular, GOAL_MIN_ANGULAR_RANGE)) {
            RCLCPP_INFO(get_logger(), "goal !");
            opti_twists_ = std::nullopt;
            opti_path_ = std::nullopt;
            cmd_vel_pub_->publish(stop());
            return true;
          }
        }
      }
#if defined(CONTROL_DEBUG_OUTPUT)
      std::cout << "control_time:" << control_time << std::endl;
      std::cout << "cmd_vel:" << cmd_vel << std::endl;
#endif
      if (
        !cmd_vel.linear.has_inf() && !cmd_vel.angular.has_inf() && !cmd_vel.linear.has_nan() &&
        !cmd_vel.angular.has_nan())
        cmd_vel_pub_->publish(make_geometry_twist(cmd_vel));
      else {
        RCLCPP_WARN(get_logger(), "error cmd_vel inf or nan !");
        std::cout << "cmd_vel:" << cmd_vel << std::endl;
        cmd_vel_pub_->publish(stop());
      }
    }
    linear_vel_pub_->publish(make_float32(vel));
    angular_vel_pub_->publish(make_float32(angular));
    perfomance_pub_->publish(make_float32(
      unit_cast<unit::time::s, unit::time::ms>((now_time - pre_control_time_).seconds())));
    pre_control_time_ = get_clock()->now();
    return false;
  }

private:
  bool control_time_over_ = false;
  bool controller_running_ = false;
  bool time_sync_ = false;
  // param
  std::string MAP_FRAME;
  std::string ROBOT_FRAME;
  double CONTROL_PERIOD;
  double MPC_DT;
  double inv_MPC_DT;
  double GOAL_POS_RANGE;
  double GOAL_ANGLE_RANGE;
  double GOAL_MIN_VEL_RANGE;
  double GOAL_MIN_ANGULAR_RANGE;
  double TIME_OUT;
  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;
  // timer
  rclcpp::Time pre_control_time_;
  rclcpp::TimerBase::SharedPtr timer_;
  // action
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  // subscriber
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr opti_path_sub_;
  rclcpp::Subscription<extension_msgs::msg::TwistMultiArray>::SharedPtr opti_twists_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr mpc_dt_sub_;
  // publisher
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr linear_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angular_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr perfomance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr control_time_pub_;
  // twist
  Twistd odom_vel_;
  // pose
  Pose3d target_pose_;
  // path
  std::optional<nav_msgs::msg::Path> opti_path_;
  std::optional<extension_msgs::msg::TwistMultiArray> opti_twists_;

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