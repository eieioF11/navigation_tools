#pragma once
#include <execution>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "mpc_path_planning/mpc_path_planning_component.hpp"

using namespace common_lib;
using namespace std::chrono_literals;
class Control : public ExtensionNode
{
public:
  Control(const rclcpp::NodeOptions & options) : Control("", options) {}
  Control(
    const std::string & name_space = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : ExtensionNode("control_node", name_space, options),
    tf_buffer_(this->get_clock()),
    listener_(tf_buffer_)
  {
    RCLCPP_INFO(this->get_logger(), "start control_node");
    // get param
    std::string OPTIPATH_TOPIC =
      param<std::string>("control.topic_name.opti_path", "mpc_path_planning/opti_path");
    std::string OPTITWISTS_TOPIC =
      param<std::string>("control.topic_name.opti_twists", "mpc_path_planning/twists");
    std::string CMD_VEL_TOPIC = param<std::string>("control.topic_name.cmd_vel", "/cmd_vel");
    // frame
    MAP_FRAME = param<std::string>("control.tf_frame.map_frame", "map");
    ROBOT_FRAME = param<std::string>("control.tf_frame.robot_frame", "base_link");
    // setup
    CONTROL_PERIOD = param<double>("control.control_period", 0.001);
    // 収束判定
    GOAL_POS_RANGE = param<double>("control.goal.pos_range", 0.01);
    GOAL_ANGLE_RANGE = unit_cast<unit::angle::rad>(param<double>("control.goal.angle_range", 0.1));
    GOAL_MIN_VEL_RANGE = param<double>("control.goal.min_vel_range", 0.001);
    GOAL_MIN_ANGULAR_RANGE = param<double>("control.goal.min_angular_range", 0.001);
    // init
    RCLCPP_INFO(this->get_logger(), "Initialization !");
    opti_twists_ = std::nullopt;
    opti_path_ = std::nullopt;
    pre_control_time_ = this->get_clock()->now();
    // publisher
    end_pub_ =
      this->create_publisher<std_msgs::msg::Empty>("mpc_path_planning/end", rclcpp::QoS(10).reliable());
    cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>(CMD_VEL_TOPIC, rclcpp::QoS(10));
    perfomance_pub_ =
      this->create_publisher<std_msgs::msg::Float32>("mpc_path_planning/control_time", rclcpp::QoS(5));
    // subscriber
    opti_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      OPTIPATH_TOPIC, rclcpp::QoS(10).reliable(),
      [&](const nav_msgs::msg::Path::SharedPtr msg) { opti_path_ = *msg; });
    opti_twists_sub_ = this->create_subscription<extension_msgs::msg::TwistMultiArray>(
      OPTITWISTS_TOPIC, rclcpp::QoS(10),
      [&](extension_msgs::msg::TwistMultiArray::SharedPtr msg) { opti_twists_ = *msg; });
    // timer
    control_timer_ = this->create_wall_timer(1s * CONTROL_PERIOD, [&]() {
      if (!tf_buffer_.canTransform(
            ROBOT_FRAME, MAP_FRAME, rclcpp::Time(0),
            tf2::durationFromSec(1.0))) {  // 変換無いよ
        RCLCPP_WARN(
          this->get_logger(), "%s %s can not Transform", MAP_FRAME.c_str(), ROBOT_FRAME.c_str());
        return;
      }
      auto map_to_base_link = lookup_transform(tf_buffer_, ROBOT_FRAME, MAP_FRAME);
      auto now_time = this->get_clock()->now();
      if (map_to_base_link && opti_path_ && opti_twists_) {
        Pathd opti_path = make_path(opti_path_.value(), opti_twists_.value());
        Pose3d base_link_pose = make_pose(map_to_base_link.value().transform);
        RCLCPP_INFO_CHANGE(0, this->get_logger(), "get opti_path");
#if defined(CONTROL_DEBUG_OUTPUT)
          std::cout << "----------------------------------------------------------" << std::endl;
#endif
        // 制御出力
        Twistd cmd_vel = make_twist(stop());
        auto duration = now_time - opti_twists_.value().header.stamp;
        double control_time = duration.seconds();
        if (control_time < 0) control_time = 0;
        size_t horizon = std::round(control_time / CONTROL_PERIOD);
        if (horizon < opti_path.points.size()) {
          auto & target_twist0 = opti_path.points[horizon].velocity;
          auto nhorizon = horizon;
          if (horizon + 1 < opti_path.points.size()) nhorizon += 1;
          auto & target_twist1 = opti_path.points[nhorizon].velocity;
#if defined(CONTROL_DEBUG_OUTPUT)
          std::cout << "control_time:" << control_time << std::endl;
          std::cout << "target_twist0:" << target_twist0 << std::endl;
          std::cout << "target_twist1:" << target_twist1 << std::endl;
#endif
          double t = (control_time - CONTROL_PERIOD * horizon) / CONTROL_PERIOD;
          if (t < 0) t = 0;
          Vector3d v0 = {target_twist0.linear.x, target_twist0.linear.y, target_twist0.angular.z};
          Vector3d v1 = {target_twist1.linear.x, target_twist1.linear.y, target_twist1.angular.z};
          Vector3d v = v0 + (v1 - v0) * t;
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
        }
        Pose3d target_pose = opti_path.points.back().pose;
        double target_dist = Vector3d::distance(target_pose.position, base_link_pose.position);
        double target_diff_angle =
          std::abs(target_pose.orientation.get_rpy().z - base_link_pose.orientation.get_rpy().z);
        double vel = cmd_vel.linear.norm();
        double angular = cmd_vel.angular.norm();
#if defined(CONTROL_DEBUG_OUTPUT)
        std::cout << "target_dist:" << target_dist << std::endl;
        std::cout << "target_diff_angle:" << target_diff_angle << std::endl;
        std::cout << "vel:" << vel << std::endl;
        std::cout << "angular:" << angular << std::endl;
#endif
        // ゴール判定
        if (target_dist < GOAL_POS_RANGE) {
          if (target_diff_angle < GOAL_ANGLE_RANGE) {
            if (
              approx_zero(vel, GOAL_MIN_VEL_RANGE) &&
              approx_zero(angular, GOAL_MIN_ANGULAR_RANGE)) {
              RCLCPP_INFO(this->get_logger(), "goal !");
              opti_twists_ = std::nullopt;
              opti_path_ = std::nullopt;
              end_pub_->publish(std_msgs::msg::Empty());
              cmd_vel_pub_->publish(stop());
              return;
            }
          }
        }
#if defined(CONTROL_DEBUG_OUTPUT)
        std::cout << "control_time:" << control_time << std::endl;
        std::cout << "horizon:" << horizon << std::endl;
        std::cout << "cmd_vel:" << cmd_vel << std::endl;
        // std::cout << "now_vel_:" << now_vel_ << std::endl;
#endif
        cmd_vel_pub_->publish(make_geometry_twist(cmd_vel));
      } else
        cmd_vel_pub_->publish(stop());
      perfomance_pub_->publish(make_float32((now_time - pre_control_time_).seconds() * 1000));
      pre_control_time_ = this->get_clock()->now();
    });
  }

private:
  // param
  std::string MAP_FRAME;
  std::string ROBOT_FRAME;
  double CONTROL_PERIOD;
  double GOAL_POS_RANGE;
  double GOAL_ANGLE_RANGE;
  double GOAL_MIN_VEL_RANGE;
  double GOAL_MIN_ANGULAR_RANGE;
  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;
  // timer
  rclcpp::TimerBase::SharedPtr path_planning_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Time pre_control_time_;
  // subscriber
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr opti_path_sub_;
  rclcpp::Subscription<extension_msgs::msg::TwistMultiArray>::SharedPtr opti_twists_sub_;
  // publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr end_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr perfomance_pub_;
  ;
  // twist
  Twistd now_vel_;
  // pose
  Pose3d old_base_link_pose_;
  //path
  std::optional<nav_msgs::msg::Path> opti_path_;
  std::optional<extension_msgs::msg::TwistMultiArray> opti_twists_;

  Vector3d Angles(Vector3d in)
  {
    Vector3d out;
    out.x = Angle(in.x);
    out.y = Angle(in.y);
    out.z = Angle(in.z);
    return out;
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