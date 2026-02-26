#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
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
#include <atomic>

// Native solver
#include <wbqp_controller/qp_solver_native.hpp>

// C++ Jacobian (header-only)
#include <wbqp_controller/wb_jac.hpp>

// ------- DEBUG STRUCTS ------- //
// In your node class (e.g., WbqpControllerNode)

enum class Mode
{
    RUN,
    DEBUG
};

struct DebugInputs
{
    // Position of the Base_link in the Neobotics frame
    std::array<double, 3> P_N2B {0.0, 0.0, 0.0};

    // Euler angles [rad]
    std::array<double, 3> theta_N2B_rad {0,0,0};
    std::array<double, 3> theta_W2N_rad {0,0,0};

    // Joint positions [rad]
    std::array<double, 6> q_rad {0,0,0,0,0,0};

    // Desired base twist u* (6)
    std::array<double, 6> u_star {0,0,0,0,0,0};

    // Warm start (9)
    std::array<double, 9> dotq_prev {0,0,0,0,0,0,0,0,0};

    // Step control for DEBUG
    bool step_once = false;
};

struct QpInput
{
    double J[54] = {0};
    double q[6] = {0};
    double u_star[6] = {0};
    double dotq_prev[9] = {0};
    double dt = 0.0;
};

struct QpConfig
{
    double beta_arm = 0.0;
    double alpha_xy = 0.0;
    double alpha_yaw = 0.0;
    double w_lin = 0.0;
    double w_ang = 0.0;
    double nu = 0.0;
    double max_dotq = 0.0;
    double max_V = 0.0;
    double max_Omegaz = 0.0;
    double qddot_max = 0.0;
    double a_lin_max = 0.0;
    double alpha_max = 0.0;
    double qmin[6] = {0};
    double qmax[6] = {0};
};

// ------- RUNTIME CONTROLLER NODE CLASS ------- //
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
    void armVelCb(const geometry_msgs::msg::Twist::SharedPtr msg);
    bool loopStep();

    // Services
    void onEnableQp(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void onEmergencyStop(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    // Native QP
    void solve_qp_native(const QpInput &in, const double J6x9_colmajor[54], double x_opt[9]);

    // C++ Jacobian (wb_jac) computation
    void computeJacobianRos2(double J6x9_colmajor[54]) const;
    void initKinematics();  // called once after params are loaded

    void fillCfgStruct(QpConfig &cfg);
    void fillInputStruct(QpInput &in, const double J6x9_colmajor[54]) const;
    void publishOutputs(const double x_opt[9]);
    bool checkXoptValues(const double x_opt[9]) const;

    // Pose integration + TF
    void integrateAndPublishBase(const double x_opt[9]);
    void publishBaseState();
    void publishStaticBaseToBaseLink();
    static geometry_msgs::msg::Quaternion quatFromRPY(double r, double p, double y);
    void publishArmTwist(const double J6x9_colmajor[54],const double qdot9[9]) const;

    // Config handlers
    void check_params();
    void print_params(const QpConfig &cfg);
    void shutdown_handler();

    // Debug mode
    void declare_and_setup_debug_params_();
    void apply_params_();
    void start_menu_thread_();
    void stop_menu_thread_();

    // Params / topics
    std::string topic_joint_state_;
    std::string topic_twist_cmd_;
    std::string topic_cmd_vel_;
    std::string topic_q_speed_;
    std::string topic_arm_vel_;
    std::string topic_arm_vel_cmd_;
    std::string topic_emergency_state_;
    std::string emergency_stop_service_name_;
    double dt_;

    std::array<double,6> in_qmin_from_cfg_or_params_{};
    std::array<double,6> in_qmax_from_cfg_or_params_{};

    // C++ Jacobian path
    bool jac_to_world_  = false;   // if true, jacobianWorld(); else jacobianBody()
    MobileManipulatorKinematics kin_;  // kinematics instance
    std::array<MobileManipulatorKinematics::DHParam, 6> dh_params_;
    std::array<double, 6> tcp_offset_ = {0,0,0,0,0,0};

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
    double arm_vel_[6]      = {0,0,0,0,0,0};  // latest arm twist from sensing
    bool have_js_           = false;
    bool have_twist_        = false;
    bool qp_enabled_        = false;
    std::atomic<bool> emergency_stop_active_{false};
    geometry_msgs::msg::PoseStamped base_pose_;
    geometry_msgs::msg::TransformStamped map_base_tf_;

    bool qp_initialized_ = true;

    // ROS I/O
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_arm_vel_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_q_speed_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_arm_vel_cmd_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_emergency_state_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_base_pose_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr qp_switch_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr emergency_stop_srv_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Executor & spinner time measurement
    rclcpp::executors::MultiThreadedExecutor executor_;
  	double spinner_mean_ = 0.0; 			  // Mean time for the spinner loop
  	unsigned long long int num_samples_ = 0;  // Number of samples for the mean time calculation

    // Debug mode
    Mode mode_ {Mode::RUN};
    bool console_menu_ = false;         // if true, run a console menu thread
    std::thread menu_thread_;
    std::atomic<bool> stop_menu_ = false;
    DebugInputs dbg_;
    std::mutex dbg_mtx_;
    using ParamCbHandle = rclcpp::node_interfaces::OnSetParametersCallbackHandle;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

};
