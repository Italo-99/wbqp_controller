#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <cstring>
#include <cmath>

// Native solver
#include <wbqp_controller/qp_solver_native.hpp>

// === MATLAB Coder headers from vendor package ===
// Use __has_include to include initialize/terminate only if present.
#include <WholeBodyJacobian.h>
#include <wbqp_init.h>
#include <wbqp_solve.h>

class WbqpControllerNode : public rclcpp::Node {
public:
    explicit WbqpControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~WbqpControllerNode();

    // Spinner
    void spinner();

private:
    // Callbacks
    void jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg);
    void twistCmdCb(const geometry_msgs::msg::Twist::SharedPtr msg);
    void loopStep();

    // Services
    void onEnableQp(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    // Native QP
    void solve_qp_native(const struct1_T &in, const double J6x9_colmajor[54], double x_opt[9]);

    // Builders for MATLAB inputs
    void buildIn15(double out_in1_15[15]) const; // [P_N2B(3), q(6), theta_N2B(3), theta_W2N(3)]
    void fillCfgStruct(struct10_T &cfg);
    void fillInputStruct(struct1_T &in, const double J6x12_colmajor[72]) const;

    // Helpers
    static void reduce_J_6x12_to_6x9(const double J6x12_colmajor[72],
                                     const std::vector<int>& cols1based,
                                     double J6x9_colmajor[54]);
    void publishOutputs(const double x_opt[9]);

    // Pose integration + TF
    void integrateAndPublishBase(const double x_opt[9]);
    void publishBaseState();
    void publishStaticBaseToBaseLink();
    static geometry_msgs::msg::Quaternion quatFromRPY(double r, double p, double y);

    // Config handlers
    void check_params();
    void print_params(const struct10_T &cfg);
    void shutdown_handler();

    // Params / topics
    std::string topic_joint_state_;
    std::string topic_twist_cmd_;
    std::string topic_cmd_vel_;
    std::string topic_q_speed_;
    double dt_;

    // Native qp
    bool use_native_qp_ = true;
    std::array<double,6> in_qmin_from_cfg_or_params_{};
    std::array<double,6> in_qmax_from_cfg_or_params_{};

    // Frames & pose state
    std::string map_frame_;
    std::string base_frame_;
    std::string base_link_frame_;
    double x_base_ = 0.0;
    double y_base_ = 0.0;

    // Kinematics for WBJ
    double P_N2B_[3];
    double theta_N2B_[3];
    geometry_msgs::msg::TransformStamped tf_N2B_;

    // State
    double q_pos_[6]        = {0,0,0,0,0,0};
    double q_vel_[6]        = {0,0,0,0,0,0};
    double theta_W2N_[3]    = {0,0,0};
    double dotq_prev_[9]    = {0,0,0,0,0,0,0,0,0};
    double u_star_[6]       = {0,0,0,0,0,0};
    bool have_js_           = false;
    bool have_twist_        = false;
    bool qp_enabled_        = false;
    geometry_msgs::msg::PoseStamped base_pose_;
    geometry_msgs::msg::TransformStamped map_base_tf_;

    // Column mapping (1-based)
    std::vector<int> cols1based_;

    // MATLAB persistent config
    struct11_T qp_{}; // output of wbqp_init
    bool qp_initialized_ = false;

    // ROS I/O
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_q_speed_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_base_pose_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr qp_switch_srv_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Executor & spinner time measurement
    rclcpp::executors::MultiThreadedExecutor executor_;
  	double spinner_mean_ = 0.0; 			        // Mean time for the spinner loop
  	unsigned long long int num_samples_ = 0;  // Number of samples for the mean time calculation
};
