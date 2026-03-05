#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
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
#include <mutex>

#include <Eigen/Core>

// Native solver
#include <wbqp_controller/qp_solver_native.hpp>

// Task-priority solver
#include <tp_control/controller.hpp>

// C++ Jacobian (header-only)
#include <wbqp_controller/wb_jac.hpp>

// MATLAB Coder headers from vendor package
#include <WholeBodyJacobian.h>
#include <wbqp_init.h>
#include <wbqp_solve.h>

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

// ------- RUNTIME CONTROLLER NODE CLASS ------- //
class WbqpControllerNode : public rclcpp::Node {
public:
    explicit WbqpControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~WbqpControllerNode();

    // Spinner
    void spinner();

private:
    enum class SingularityMethod
    {
        DET,
        YOSHIKAWA
    };

    // Callbacks
    void jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg);
    void twistCmdCb(const geometry_msgs::msg::Twist::SharedPtr msg);
    void poseGoalCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void armVelCb(const geometry_msgs::msg::Twist::SharedPtr msg);
    void obstaclePointsCb(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    bool loopStep();

    // Services
    void onEnableQp(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void onEmergencyStop(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    // Native QP
    void solve_qp_native(const struct1_T &in, const double J6x9_colmajor[54], double x_opt[9]);
    void initTaskPriorityController();
    void solve_tp(double x_opt[9]);
    tp_control::Pose6D computeCurrentEePoseTp(const tp_control::RobotState& state) const;

    // C++ Jacobian (wb_jac) computation
    void computeJacobianRos2(double J6x9_colmajor[54]) const;
    void initKinematics();  // called once after params are loaded

    void buildIn15(double out_in1_15[15]) const; // [P_N2B(3), q(6), theta_N2B(3), theta_W2N(3)]
    void fillCfgStruct(struct10_T &cfg);
    void fillInputStruct(struct1_T &in, const double J6x12_colmajor[72]) const;
    static void reduce_J_6x12_to_6x9(const double J6x12_colmajor[72],
                                     const std::vector<int>& cols1based,
                                     double J6x9_colmajor[54]);
    void enforceSingularityConstraint(double x_opt[9]);
    double computeSingularityMetricForQ(const std::array<double,6>& q_rad) const;
    double computeSingularityMetricFromArmJacobian(const Eigen::Matrix<double,6,6>& J_arm) const;
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
    void print_params(const struct10_T &cfg);
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
    std::string topic_tcp_pose_goal_;
    std::string topic_emergency_state_;
    std::string emergency_stop_service_name_;
    double dt_;
    bool tp_method_ = false;

    enum class TpTrackingMode
    {
        SPEED,
        POSE
    };
    enum class TpPoseSubMode
    {
        INCREMENT,
        INPUT
    };

    bool use_native_qp_ = true;
    std::array<double,6> in_qmin_from_cfg_or_params_{};
    std::array<double,6> in_qmax_from_cfg_or_params_{};
    // Runtime native-QP tuning loaded from params (qp.*).
    double qp_w_lin_ = 1.0;
    double qp_w_ang_ = 1.0;
    double qp_beta_arm_ = 0.001;
    double qp_alpha_xy_ = 0.002;
    double qp_alpha_yaw_ = 0.002;
    double qp_nu_ = 0.001;
    double qp_max_dotq_ = 1.0;
    double qp_max_V_ = 0.2;
    double qp_max_Omegaz_ = 0.2;
    double qp_qddot_max_ = 2.0;
    double qp_a_lin_max_ = 1.0;
    double qp_alpha_max_ = 1.0;

    // QP-native method parameters (qp.*)
    bool qp_singularity_enable_ = false;
    double qp_singularity_min_threshold_ = 1e-4;
    SingularityMethod qp_singularity_method_ = SingularityMethod::YOSHIKAWA;
    std::string qp_singularity_method_name_ = "yoshikawa";
    bool qp_collision_enable_ = false;
    double qp_collision_d_safe_ = 0.15;
    int qp_collision_max_constraints_ = 128;
    double qp_collision_base_size_x_ = 0.70;
    double qp_collision_base_size_y_ = 0.50;
    double qp_collision_base_delta_h_ = 0.05;
    int qp_collision_samples_per_link_ = 0;
    bool qp_collision_include_tcp_ = true;
    bool qp_collision_include_joint_endpoints_ = true;
    bool qp_collision_use_closest_obstacle_ = true;
    bool qp_tcp_z_guard_enable_ = false;
    double qp_tcp_z_guard_z_min_ = 0.03;

    // TP method parameters (tp.*)
    double tp_solver_lambda_ = 1.0e-4;
    std::string tp_solver_pinv_method_ = "svd";
    int tp_priority_tracking_ = 1;
    int tp_priority_joint_limits_ = 2;
    int tp_priority_tcp_z_ = 3;
    int tp_priority_singularity_ = 4;
    int tp_priority_collision_ = 5;
    std::array<double,6> tp_tracking_kp_ {2,2,2,2,2,2};
    std::array<double,6> tp_tracking_ki_ {0,0,0,0,0,0};
    std::array<double,6> tp_tracking_v_limit_ {1,1,1,1,1,1};
    double tp_joint_limits_k_ = 2.0;
    double tp_joint_limits_d_act_ = 0.2;
    double tp_joint_limits_margin_ = 0.05;
    bool tp_singularity_enable_ = false;
    bool tp_collision_enable_ = false;
    bool tp_tcp_z_guard_enable_ = false;
    double tp_tcp_z_guard_z_min_ = 0.03;
    double tp_tcp_z_guard_z_act_ = 0.06;
    double tp_tcp_z_guard_k_ = 2.0;
    double tp_collision_d_safe_ = 0.15;
    double tp_collision_d_act_ = 0.30;
    double tp_collision_k_ = 3.0;
    int tp_collision_max_constraints_ = 64;
    double tp_collision_base_size_x_ = 0.70;
    double tp_collision_base_size_y_ = 0.50;
    double tp_collision_base_delta_h_ = 0.05;
    int tp_collision_samples_per_link_ = 0;
    bool tp_collision_include_tcp_ = true;
    bool tp_collision_include_joint_endpoints_ = true;
    bool tp_collision_use_closest_obstacle_ = true;
    std::string tp_singularity_method_ = "yoshikawa";
    double tp_singularity_mu_min_ = 1.0e-4;
    double tp_singularity_mu_max_ = 1.25e-4;
    double tp_singularity_mu_safe_ = 1.5e-4;
    double tp_singularity_k_ = 1.0;
    double tp_singularity_fd_eps_ = 1.0e-4;
    TpTrackingMode tp_tracking_mode_ = TpTrackingMode::SPEED;
    TpPoseSubMode tp_pose_sub_mode_ = TpPoseSubMode::INCREMENT;
    std::string topic_obstacle_points_;

    std::vector<int> cols1based_;

    // C++ Jacobian path
    bool use_jac_ros2_  = false;   // if true, use MobileManipulatorKinematics instead of MATLAB codegen
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
    bool have_pose_goal_input_ = false;
    bool tp_pose_target_initialized_ = false;
    bool qp_enabled_        = false;
    std::atomic<bool> emergency_stop_active_{false};
    geometry_msgs::msg::PoseStamped base_pose_;
    geometry_msgs::msg::TransformStamped map_base_tf_;

    struct11_T qp_{}; // output of wbqp_init
    bool qp_initialized_ = false;
    std::unique_ptr<tp_control::TaskPriorityController> tp_controller_;

    // ROS I/O
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_goal_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_arm_vel_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_obstacles_;
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
    std::vector<Eigen::Vector3d> obstacle_points_world_;
    std::string obstacle_points_frame_;
    mutable std::mutex obstacle_points_mtx_;
    tp_control::Pose6D tp_pose_goal_input_;
    tp_control::Pose6D tp_pose_goal_target_;
    mutable std::mutex tp_pose_goal_mtx_;
    DebugInputs dbg_;
    std::mutex dbg_mtx_;
    using ParamCbHandle = rclcpp::node_interfaces::OnSetParametersCallbackHandle;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

};
