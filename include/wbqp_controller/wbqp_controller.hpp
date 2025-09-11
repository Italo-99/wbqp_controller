#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

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

// === MATLAB Coder headers from vendor package ===
// Use __has_include to include initialize/terminate only if present.
#include <WholeBodyJacobian.h>
#include <wbqp_init.h>
#include <wbqp_solve.h>
#if __has_include(<wbqp_init_initialize.h>)
  #include <wbqp_init_initialize.h>
  #define HAS_WBQP_INIT_INIT 1
#endif
#if __has_include(<wbqp_init_terminate.h>)
  #include <wbqp_init_terminate.h>
  #define HAS_WBQP_INIT_TERM 1
#endif
#if __has_include(<wbqp_solve_initialize.h>)
  #include <wbqp_solve_initialize.h>
  #define HAS_WBQP_SOLVE_INIT 1
#endif
#if __has_include(<wbqp_solve_terminate.h>)
  #include <wbqp_solve_terminate.h>
  #define HAS_WBQP_SOLVE_TERM 1
#endif
#if __has_include(<WholeBodyJacobian_initialize.h>)
  #include <WholeBodyJacobian_initialize.h>
  #define HAS_WBJ_INIT 1
#endif
#if __has_include(<WholeBodyJacobian_terminate.h>)
  #include <WholeBodyJacobian_terminate.h>
  #define HAS_WBJ_TERM 1
#endif

// Types headers (optional, only if you need struct0_T/struct1_T/struct2_T definitions)
#if __has_include(<wbqp_init_types.h>)
  #include <wbqp_init_types.h>
#elif __has_include(<wbqp_solve_types.h>)
  #include <wbqp_solve_types.h>
#else
  // If types are not in separate headers, they are usually in main headers above.
#endif

class WbqpControllerNode : public rclcpp::Node {
public:
    explicit WbqpControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~WbqpControllerNode();

private:
    // Callbacks
    void jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg);
    void twistCmdCb(const geometry_msgs::msg::Twist::SharedPtr msg);
    void loopStep();

    // Builders for MATLAB inputs
    void buildIn15(double out_in1_15[15]) const; // [P_N2B(3), q(6), theta_N2B(3), theta_W2N(3)]
    void fillCfgStruct(struct0_T &cfg);
    void fillInputStruct(struct1_T &in, const double J6x12_colmajor[72]) const;

    // Helpers
    static void reduce_J_6x12_to_6x9(const double J6x12_colmajor[72],
                                     const std::vector<int> &cols1based,
                                     double J6x9_colmajor[54]);
    void publishOutputs(const double x_opt[9]);

    // Pose integration + TF
    void integrateAndPublishBase(const double x_opt[9]);
    void publishStaticBaseToBaseLink();
    static geometry_msgs::msg::Quaternion quatFromRPY(double r, double p, double y);

    // Params / topics
    std::string topic_joint_state_;
    std::string topic_twist_cmd_;
    std::string topic_cmd_vel_;
    std::string topic_q_speed_;
    double dt_;

    // Frames & pose state
    std::string map_frame_;
    std::string base_frame_;
    std::string base_link_frame_;
    double x_base_ = 0.0;
    double y_base_ = 0.0;

    // Kinematics for WBJ
    double P_N2B_[3];
    double theta_N2B_[3];

    // State
    double q_pos_[6]        = {0,0,0,0,0,0};
    double q_vel_[6]        = {0,0,0,0,0,0};
    double theta_W2N_[3]    = {0,0,0};
    double dotq_prev_[9]    = {0,0,0,0,0,0,0,0,0};
    double u_star_[6]       = {0,0,0,0,0,0};
    bool have_js_           = false;
    bool have_twist_        = false;

    // Column mapping (1-based)
    std::vector<int> cols1based_;

    // MATLAB persistent config
    struct1_T qp_{}; // output of wbqp_init
    bool qp_initialized_ = false;

    // ROS I/O
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_q_speed_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_base_pose_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};
