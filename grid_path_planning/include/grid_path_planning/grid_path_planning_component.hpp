#pragma once
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <utility>
#include <execution>
#include <mutex>
#include <optional>
// Eigen
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
// ROS
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <laser_geometry/laser_geometry.hpp>

#include "extension_node/extension_node.hpp"
#include "common_lib/ros2_utility/ros_pcl_util.hpp"
#include "common_lib/ros2_utility/msg_util.hpp"
#include "common_lib/ros2_utility/tf_util.hpp"
// #include "common_lib/ros2_utility/marker_util.hpp"
// PCL
#include "common_lib/pcl_utility/pcl_util.hpp"
// other
#include "common_lib/common_lib.hpp"

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
class GridPathPlanning : public ExtensionNode
{
public:
	GridPathPlanning(const rclcpp::NodeOptions &options) : GridPathPlanning("", options) {}
	GridPathPlanning(const std::string &name_space = "", const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : ExtensionNode("grid_path_planning_node", name_space, options), broadcaster_(this), tf_buffer_(this->get_clock()), listener_(tf_buffer_)
	{
		RCLCPP_INFO(this->get_logger(), "start grid_path_planning_node");
		// get param
		std::string ESTIMATE_POSE_TOPIC = param<std::string>("grid_path_planning.topic_name.estimate_pose", "grid_path_planning/estimate_pose");
		std::string ESTIMATE_TWIST_TOPIC = param<std::string>("grid_path_planning.topic_name.estimate_twist", "grid_path_planning/estimate_twist");
		// frame
		MAP_FRAME = param<std::string>("grid_path_planning.tf_frame.map_frame", "map");
		ODOM_FRAME = param<std::string>("grid_path_planning.tf_frame.odom_frame", "odom");
		ROBOT_FRAME = param<std::string>("grid_path_planning.tf_frame.robot_frame", "base_link");
		// setup
		BROADCAST_PERIOD = param<double>("grid_path_planning.broadcast_period", 0.001);
		// robotパラメータ
		MIN_VEL = param<double>("grid_path_planning.robot.min_velocity", 0.01);
		MIN_ANGULAR = param<double>("grid_path_planning.robot.min_angular", 0.01);
		gpo_param.max_velocity = param<double>("grid_path_planning.robot.max_velocity", 5.0);
		gpo_param.max_angular = param<double>("grid_path_planning.robot.max_angular", 5.0);
		// init
		initialization_ = true;
		// publisher
		// estimate_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(ESTIMATE_POSE_TOPIC, rclcpp::QoS(10).best_effort());
		// estimate_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(ESTIMATE_TWIST_TOPIC, rclcpp::QoS(10).best_effort());
		// subscriber
		est_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(ESTIMATE_POSE_TOPIC, rclcpp::QoS(10).best_effort(), std::bind(&GridPathPlanning::est_pose_callback, this, std::placeholders::_1));
		est_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(ESTIMATE_TWIST_TOPIC, rclcpp::QoS(10).best_effort(), std::bind(&GridPathPlanning::est_twist_callback, this, std::placeholders::_1));
		// timer
		main_timer_ = this->create_wall_timer(1s * BROADCAST_PERIOD, [&]() {});
	}

	// callback

	void est_twist_callback(const geometry_msgs::msg::TwistStamped::ConstPtr msg)
	{
		estimate_twist_ = make_twist(msg->twist);
	}

	void est_pose_callback(const geometry_msgs::msg::PoseStamped::ConstPtr msg)
	{
		estimate_pose_ = make_pose(msg->pose);
	}

private:
	bool initialization_;
	// param
	std::string MAP_FRAME;
	std::string ROBOT_FRAME;
	std::string ODOM_FRAME;
	double BROADCAST_PERIOD;
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
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr est_pose_sub_;
	rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr est_twist_sub_;
	// publisher
	// rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimate_pose_pub_;
	// rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr estimate_twist_pub_;
	// pose
	Pose3d odom_pose_;
	Pose3d imu_pose_;
	Pose3d estimate_pose_;
	// vel
	Twistd odom_twist_;
	Twistd estimate_twist_;
};