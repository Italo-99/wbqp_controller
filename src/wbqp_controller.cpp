/*
	MIT License

	Copyright (c) [2024] [Andrea Pupa] [Italo Almirante]

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/

#include "wbqp_controller/wbqp_controller.hpp"

// ------------------- CONSTRUCTOR & DESTRUCTOR ------------------- //
WbqpControllerNode::WbqpControllerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("wbqp_controller", options)
{
    // --- Params ---
    check_params();

    // Build cfg and run wbqp_init once
    struct10_T cfg{};
    fillCfgStruct(cfg);
    wbqp_init(&cfg, &qp_);
    qp_initialized_ = true;

    print_params(cfg);

    // ROS I/O
    rclcpp::SubscriptionOptions sub_options;

    auto cb_group_sub_js = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = cb_group_sub_js;
    sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
        topic_joint_state_, 1, std::bind(&WbqpControllerNode::jointStateCb, this, std::placeholders::_1), sub_options);

    auto cb_group_sub_twist = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = cb_group_sub_twist;
    sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
        topic_twist_cmd_, 1, std::bind(&WbqpControllerNode::twistCmdCb, this, std::placeholders::_1), sub_options);

    pub_cmd_vel_  = this->create_publisher<geometry_msgs::msg::Twist>(topic_cmd_vel_, 1);
    pub_q_speed_  = this->create_publisher<sensor_msgs::msg::JointState>(topic_q_speed_, 1);
    pub_arm_vel_  = this->create_publisher<geometry_msgs::msg::Twist>(topic_arm_vel_, 1);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    pub_base_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mobile_manipulator/base_pose", 1);

    qp_switch_srv_ = this->create_service<std_srvs::srv::SetBool>("~/enable_qp",
            std::bind(&WbqpControllerNode::onEnableQp, this, std::placeholders::_1, std::placeholders::_2));

    // Publish static TF base -> base_link once
    publishStaticBaseToBaseLink();

    // ---------------- SHUTDOWN HANDLER ----------------
    rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
        std::bind(&WbqpControllerNode::shutdown_handler, this) // Register shutdown handler
    );

    // Confirm correct node startup
    RCLCPP_INFO(this->get_logger(), "Whole Body Controller node initialized successfully.");

    // --- Debug mode ---
    declare_and_setup_debug_params_();
}

WbqpControllerNode::~WbqpControllerNode()
{
    timer_.reset();
    stop_menu_thread_();
    param_cb_handle_.reset();
}

void WbqpControllerNode::check_params()
{
  // --- Topics & timing ---
  this->declare_parameter("topics.joint_state", "/joint_states");
  this->declare_parameter("topics.twist_cmd",   "/mobile_manipulator/cmd_vel");
  this->declare_parameter("topics.cmd_vel",     "/cmd_vel");
  this->declare_parameter("topics.q_speed",     "/manipulator/js_cmd_vel");
  this->declare_parameter("topics.arm_vel",     "/manipulator/cmd_vel");
  this->declare_parameter("control.dt",          0.02);

  topic_joint_state_ = this->get_parameter("topics.joint_state").as_string();
  topic_twist_cmd_   = this->get_parameter("topics.twist_cmd").as_string();
  topic_cmd_vel_     = this->get_parameter("topics.cmd_vel").as_string();
  topic_q_speed_     = this->get_parameter("topics.q_speed").as_string();
  topic_arm_vel_     = this->get_parameter("topics.arm_vel").as_string();
  dt_                = this->get_parameter("control.dt").as_double();

  // --- Frames ---
  this->declare_parameter("frames.map",       "map");
  this->declare_parameter("frames.base",      "base");
  this->declare_parameter("frames.base_link", "base_link");
  map_frame_       = this->get_parameter("frames.map").as_string();
  base_frame_      = this->get_parameter("frames.base").as_string();
  base_link_frame_ = this->get_parameter("frames.base_link").as_string();

  // --- Solver column selection (1-based) ---
  this->declare_parameter("qp.cols", std::vector<int64_t>{1,2,3,4,5,6,7,8,12});
  auto cols_i64 = this->get_parameter("qp.cols").as_integer_array();
  cols1based_.resize(cols_i64.size());
  std::transform(cols_i64.begin(), cols_i64.end(), cols1based_.begin(),
                 [](int64_t v){ return static_cast<int>(v); });

  // --- Robot fixed transform N->B ---
  this->declare_parameter("kinematics.P_N2B",     std::vector<double>{0.187, 0.0, 0.22});
  this->declare_parameter("kinematics.theta_N2B", std::vector<double>{0.0,   0.0,  0.0});
  auto p_n2b  = this->get_parameter("kinematics.P_N2B").as_double_array();
  auto th_n2b = this->get_parameter("kinematics.theta_N2B").as_double_array();
  for (int i=0;i<3;++i) { P_N2B_[i] = p_n2b[i]; theta_N2B_[i] = th_n2b[i]; }

  // --- Qp solver type ---
  this->declare_parameter<bool>("qp.use_native", true);
  use_native_qp_ = this->get_parameter("qp.use_native").as_bool();

  // Log correctly loaded params
  RCLCPP_INFO(this->get_logger(), "Parameters loaded.");
}

void WbqpControllerNode::print_params(const struct10_T &cfg)
{
    RCLCPP_INFO(this->get_logger(), "--- Loaded Parameters ---");

    // --- Topics & timing ---
    RCLCPP_INFO(this->get_logger(), "Topics:");
    RCLCPP_INFO(this->get_logger(), "  joint_state : %s", topic_joint_state_.c_str());
    RCLCPP_INFO(this->get_logger(), "  twist_cmd   : %s", topic_twist_cmd_.c_str());
    RCLCPP_INFO(this->get_logger(), "  cmd_vel     : %s", topic_cmd_vel_.c_str());
    RCLCPP_INFO(this->get_logger(), "  q_speed     : %s", topic_q_speed_.c_str());
    RCLCPP_INFO(this->get_logger(), "Control dt = %.4f", dt_);

    // --- Frames ---
    RCLCPP_INFO(this->get_logger(), "Frames:");
    RCLCPP_INFO(this->get_logger(), "  map       : %s", map_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  base      : %s", base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  base_link : %s", base_link_frame_.c_str());

    // --- Transform ---
    RCLCPP_INFO(this->get_logger(), "P_N2B     = [%.3f, %.3f, %.3f]",
                P_N2B_[0], P_N2B_[1], P_N2B_[2]);
    RCLCPP_INFO(this->get_logger(), "theta_N2B = [%.3f, %.3f, %.3f]",
                theta_N2B_[0], theta_N2B_[1], theta_N2B_[2]);

    // --- Solver columns ---
    std::ostringstream cols_ss;
    cols_ss << "[";
    for (size_t i=0; i<cols1based_.size(); ++i) {
        cols_ss << cols1based_[i];
        if (i+1 < cols1based_.size()) { cols_ss << ", "; }
    }
    cols_ss << "]";
    RCLCPP_INFO(this->get_logger(), "QP cols = %s", cols_ss.str().c_str());

    // --- QP weights & limits ---
    RCLCPP_INFO(this->get_logger(), "QP Parameters:");
    RCLCPP_INFO(this->get_logger(), "  beta_arm   = %.4f", cfg.beta_arm);
    RCLCPP_INFO(this->get_logger(), "  alpha_xy   = %.4f", cfg.alpha_xy);
    RCLCPP_INFO(this->get_logger(), "  alpha_yaw  = %.4f", cfg.alpha_yaw);
    RCLCPP_INFO(this->get_logger(), "  w_lin      = %.4f", cfg.w_lin);
    RCLCPP_INFO(this->get_logger(), "  w_ang      = %.4f", cfg.w_ang);
    RCLCPP_INFO(this->get_logger(), "  nu         = %.4f", cfg.nu);
    RCLCPP_INFO(this->get_logger(), "  max_dotq   = %.4f", cfg.max_dotq);
    RCLCPP_INFO(this->get_logger(), "  max_V      = %.4f", cfg.max_V);
    RCLCPP_INFO(this->get_logger(), "  max_Omegaz = %.4f", cfg.max_Omegaz);
    RCLCPP_INFO(this->get_logger(), "  qddot_max  = %.4f", cfg.qddot_max);
    RCLCPP_INFO(this->get_logger(), "  a_lin_max  = %.4f", cfg.a_lin_max);
    RCLCPP_INFO(this->get_logger(), "  alpha_max  = %.4f", cfg.alpha_max);

    // --- Joint limits ---
    std::ostringstream qmin_ss, qmax_ss;
    qmin_ss << "[";
    qmax_ss << "[";
    for (int i=0; i<6; ++i) {
        qmin_ss << cfg.qmin[i];
        qmax_ss << cfg.qmax[i];
        if (i<5) { qmin_ss << ", "; qmax_ss << ", "; }
    }
    qmin_ss << "]";
    qmax_ss << "]";
    RCLCPP_INFO(this->get_logger(), "qmin = %s", qmin_ss.str().c_str());
    RCLCPP_INFO(this->get_logger(), "qmax = %s", qmax_ss.str().c_str());

    RCLCPP_INFO(this->get_logger(), "--------------------------");
}

// ------------------- MAIN LOOP ------------------- //
bool WbqpControllerNode::loopStep()
{
    // Always publish static TF
    static_tf_broadcaster_->sendTransform(tf_N2B_);

    if (mode_ == Mode::RUN)
    {
        if (!qp_initialized_ || !have_js_ || !have_twist_ || !qp_enabled_)
        {
            base_pose_.header.stamp   = this->get_clock()->now();
            map_base_tf_.header.stamp = base_pose_.header.stamp;
            publishBaseState();
            return false;
        }
    }
    else
    {
        // DEBUG mode: wait for step
        {
            std::lock_guard<std::mutex> lk(dbg_mtx_);
            if (!dbg_.step_once)
            {
                // Still publish pose/TF to keep RViz sane
                base_pose_.header.stamp   = this->get_clock()->now();
                map_base_tf_.header.stamp = base_pose_.header.stamp;
                publishBaseState();
                return false;
            }
                
            dbg_.step_once = false;   // consume the step
        }

        // In DEBUG, synthesize your inputs from dbg_
        {
            std::lock_guard<std::mutex> lk(dbg_mtx_);

            P_N2B_[0] = dbg_.P_N2B[0];
            P_N2B_[1] = dbg_.P_N2B[1];
            P_N2B_[2] = dbg_.P_N2B[2];

            for (int i=0;i<6;++i)
            {
                q_pos_[i]   = dbg_.q_rad[i];
                u_star_[i]  = dbg_.u_star[i];
            }

            for (int i=0;i<9;++i)
            {
                dotq_prev_[i] = dbg_.dotq_prev[i];
            }

            for (int i=0;i<3;++i)
            {
                theta_N2B_[i] = dbg_.theta_N2B_rad[i];
                theta_W2N_[i] = dbg_.theta_W2N_rad[i];
            }

            qp_initialized_ = true;
            have_js_        = true;
            have_twist_     = true;
            qp_enabled_     = true;
        }
    }

    // --- Your existing pipeline ---
    double in1[15];
    buildIn15(in1);

    double A6x12_colmajor[72];
    WholeBodyJacobian(in1, A6x12_colmajor);

    struct1_T in{};
    fillInputStruct(in, A6x12_colmajor);

    double x_opt[9];
    if (use_native_qp_)
    {
        solve_qp_native(in, in.J, x_opt);
    }
    else
    {
        struct2_T dbg_out{};
        wbqp_solve(reinterpret_cast<const struct0_T *>(&qp_), &in, x_opt, &dbg_out);
    }

    bool xopt_ok = checkXoptValues(x_opt);
    if (!xopt_ok)
    {
        RCLCPP_ERROR(this->get_logger(), "QP solver returned invalid values (inf or nan). Skipping this cycle.");
        for (int i=0;i<9;++i) { x_opt[i] = dotq_prev_[i]; }
    }

    for (int i=0;i<9;++i) { dotq_prev_[i] = x_opt[i]; }

    // Display results if in DEBUG
    if (mode_ == Mode::DEBUG)
    {
        RCLCPP_INFO(this->get_logger(), "QP solution:");
        for (int i=0;i<9;++i) { RCLCPP_INFO(this->get_logger(), "  x_opt[%d] = %.6f", i, x_opt[i]);}
    }

    integrateAndPublishBase(x_opt);
    publishOutputs(x_opt);
    publishArmTwist(in.J, x_opt);

    return true;
}

// ------------------- CALLBACKS ------------------- //
void WbqpControllerNode::jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->position.size() >= 6) {
        for (int i=0;i<6;++i) { q_pos_[i] = msg->position[i]; }
    }
    if (msg->velocity.size() >= 6) {
        for (int i=0;i<6;++i) { q_vel_[i] = msg->velocity[i]; }
    }
    have_js_ = true;
}

void WbqpControllerNode::twistCmdCb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    u_star_[0]=msg->linear.x;  u_star_[1]=msg->linear.y;  u_star_[2]=msg->linear.z;
    u_star_[3]=msg->angular.x; u_star_[4]=msg->angular.y; u_star_[5]=msg->angular.z;
    have_twist_ = true;
}

void WbqpControllerNode::onEnableQp(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                         std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    qp_enabled_ = request->data;   // true = enable, false = disable
    response->success = true;
    response->message = qp_enabled_ ? "QP enabled" : "QP disabled";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

// ------------------- MATLAB OPTIMIZER CONFIGURATION ------------------- //
void WbqpControllerNode::fillCfgStruct(struct10_T &cfg)
{
    std::memset(&cfg, 0, sizeof(cfg));

    // cols
    for (int i=0;i<9 && i<(int)cols1based_.size(); ++i) { cfg.cols[i] = static_cast<int>(cols1based_[i]); }

    // --- QP weights & limits (scalars) ---
    this->declare_parameter("qp.beta_arm",   0.001);
    this->declare_parameter("qp.alpha_xy",   0.001);
    this->declare_parameter("qp.alpha_yaw",  0.001);
    this->declare_parameter("qp.w_lin",      1.0);
    this->declare_parameter("qp.w_ang",      1.0);
    this->declare_parameter("qp.nu",         0.001);
    this->declare_parameter("qp.max_dotq",   1.0);
    this->declare_parameter("qp.max_V",      0.5);
    this->declare_parameter("qp.max_Omegaz", 0.5);
    this->declare_parameter("qp.qddot_max",  2.0);
    this->declare_parameter("qp.a_lin_max",  1.0);
    this->declare_parameter("qp.alpha_max",  1.0);

    cfg.beta_arm   = this->get_parameter("qp.beta_arm").as_double();
    cfg.alpha_xy   = this->get_parameter("qp.alpha_xy").as_double();
    cfg.alpha_yaw  = this->get_parameter("qp.alpha_yaw").as_double();
    cfg.w_lin      = this->get_parameter("qp.w_lin").as_double();
    cfg.w_ang      = this->get_parameter("qp.w_ang").as_double();
    cfg.nu         = this->get_parameter("qp.nu").as_double();
    cfg.max_dotq   = this->get_parameter("qp.max_dotq").as_double();
    cfg.max_V      = this->get_parameter("qp.max_V").as_double();
    cfg.max_Omegaz = this->get_parameter("qp.max_Omegaz").as_double();
    cfg.qddot_max  = this->get_parameter("qp.qddot_max").as_double();
    cfg.a_lin_max  = this->get_parameter("qp.a_lin_max").as_double();
    cfg.alpha_max  = this->get_parameter("qp.alpha_max").as_double();

    // --- Joint limits arrays ---
    this->declare_parameter("limits.qmin", std::vector<double>{-3.14,-3.14,-3.14,-3.14,-3.14,-3.14});
    this->declare_parameter("limits.qmax", std::vector<double>{ 3.14, 3.14, 3.14, 3.14, 3.14, 3.14});
    auto qmin = this->get_parameter("limits.qmin").as_double_array();
    auto qmax = this->get_parameter("limits.qmax").as_double_array();
    for (int i=0;i<6;++i) { cfg.qmin[i] = qmin[i]; cfg.qmax[i] = qmax[i]; }

    // Native qp solver: store qmin/qmax for warm start
    for (int i=0;i<6;++i) {
        in_qmin_from_cfg_or_params_[i] = cfg.qmin[i];
        in_qmax_from_cfg_or_params_[i] = cfg.qmax[i];
    }
}

void WbqpControllerNode::fillInputStruct(struct1_T &in, const double J6x12_colmajor[72]) const
{
    std::memset(&in, 0, sizeof(in));

    // Provide reduced J (6x9) if your generated struct expects field 'J'
    double J6x9[54];
    reduce_J_6x12_to_6x9(J6x12_colmajor, cols1based_, J6x9);
    for (int i=0;i<54;++i) { in.J[i] = J6x9[i]; }

    for (int i=0;i<6;++i) { in.q[i] = q_pos_[i]; }
    for (int i=0;i<6;++i) { in.u_star[i] = u_star_[i]; }
    for (int i=0;i<9;++i) { in.dotq_prev[i] = dotq_prev_[i]; }
    in.dt = dt_;
}

void WbqpControllerNode::buildIn15(double out_in1_15[15]) const
{
    out_in1_15[0]=P_N2B_[0]; out_in1_15[1]=P_N2B_[1]; out_in1_15[2]=P_N2B_[2];
    for (int i=0;i<6;++i){ out_in1_15[3+i] = q_pos_[i]; }
    out_in1_15[9]=theta_N2B_[0]; out_in1_15[10]=theta_N2B_[1]; out_in1_15[11]=theta_N2B_[2];
    out_in1_15[12]=theta_W2N_[0]; out_in1_15[13]=theta_W2N_[1]; out_in1_15[14]=theta_W2N_[2];
}

void WbqpControllerNode::reduce_J_6x12_to_6x9(const double J6x12_colmajor[72],
                                              const std::vector<int>& cols1based,
                                              double J6x9_colmajor[54])
{
    for (int j=0;j<9;++j) {
        int src_c = cols1based[j] - 1;
        for (int r=0;r<6;++r) {
            J6x9_colmajor[r + 6*j] = J6x12_colmajor[r + 6*src_c];
        }
    }
}

// ------------------- NATIVE OPTIMIZER CONFIGURATION ------------------- //
void WbqpControllerNode::solve_qp_native(const struct1_T &in, const double J6x9_colmajor[54], double x_opt[9])
{
    wbqp::NativeQpInput nin;
    // copy J
    for (int i=0;i<54;++i) nin.J6x9_colmajor[i] = J6x9_colmajor[i];
    // copy u*, q, dotq_prev, dt, q-limits
    for (int i=0;i<6;++i) { nin.u_star[i] = in.u_star[i]; nin.q[i] = in.q[i]; nin.qmin[i] = in_qmin_from_cfg_or_params_[i]; nin.qmax[i] = in_qmax_from_cfg_or_params_[i]; }
    for (int i=0;i<9;++i) nin.dotq_prev[i] = in.dotq_prev[i];
    nin.dt = in.dt;

    wbqp::NativeQpOutput zout = wbqp::NativeQpSolver::solve(nin);
    for (int i=0;i<9;++i) x_opt[i] = zout[i];
}

// ----------------- MOBILE BASE ODOMETRY ----------------------- //
void WbqpControllerNode::integrateAndPublishBase(const double x_opt[9])
{
    // Extract base state
    const double Vx_b = x_opt[6];
    const double Vy_b = x_opt[7];
    const double wz   = x_opt[8];

    // Update base pose
    // x_base_    += Vx_b * dt_;
    // y_base_    += Vy_b * dt_;
    theta_W2N_[2] += wz   * dt_;
    theta_W2N_[2] = std::atan2(std::sin(theta_W2N_[2]), std::cos(theta_W2N_[2]));

    // Convert results in world frame
    const double th   = theta_W2N_[2];
    const double c    = std::cos(th);
    const double s    = std::sin(th);

    // Compute x and y velocities
    const double vx_w = +c * Vx_b + s * Vy_b;
    const double vy_w = -s * Vx_b + c * Vy_b;
    x_base_          += vx_w * dt_;
    y_base_          += vy_w * dt_;

    // Publish results
    publishBaseState();
}

void WbqpControllerNode::publishBaseState()
{
    // Publish base pose
    base_pose_.header.stamp     = this->get_clock()->now();
    base_pose_.header.frame_id  = map_frame_;
    base_pose_.pose.position.x  = x_base_;
    base_pose_.pose.position.y  = y_base_;
    base_pose_.pose.position.z  = 0.0;
    base_pose_.pose.orientation = quatFromRPY(0.0, 0.0, theta_W2N_[2]);
    pub_base_pose_->publish(base_pose_);

    // Publish TF map -> base
    map_base_tf_.header.stamp               = base_pose_.header.stamp;
    map_base_tf_.header.frame_id            = map_frame_;
    map_base_tf_.child_frame_id             = base_frame_;
    map_base_tf_.transform.translation.x    = x_base_;
    map_base_tf_.transform.translation.y    = y_base_;
    map_base_tf_.transform.translation.z    = 0.0;
    map_base_tf_.transform.rotation         = base_pose_.pose.orientation;
    tf_broadcaster_->sendTransform(map_base_tf_);
}

// ---------------------------- HELPERS ---------------------------- //
void WbqpControllerNode::publishStaticBaseToBaseLink()
{
    tf_N2B_.header.stamp = this->get_clock()->now();
    tf_N2B_.header.frame_id = base_frame_;
    tf_N2B_.child_frame_id  = base_link_frame_;
    tf_N2B_.transform.translation.x = P_N2B_[0];
    tf_N2B_.transform.translation.y = P_N2B_[1];
    tf_N2B_.transform.translation.z = P_N2B_[2];
    tf_N2B_.transform.rotation = quatFromRPY(theta_N2B_[0], theta_N2B_[1], theta_N2B_[2]);
    static_tf_broadcaster_->sendTransform(tf_N2B_);
}

void WbqpControllerNode::publishOutputs(const double x_opt[9])
{
    geometry_msgs::msg::Twist base;
    base.linear.x  = x_opt[6];
    base.linear.y  = x_opt[7];
    base.angular.z = x_opt[8];
    pub_cmd_vel_->publish(base);

    // When you want to publish:
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.velocity.resize(6);
    for (int i = 0; i < 6; ++i) {js.velocity[i] = x_opt[i];}
    pub_q_speed_->publish(js);
}

geometry_msgs::msg::Quaternion WbqpControllerNode::quatFromRPY(double r, double p, double y)
{
    tf2::Quaternion q; q.setRPY(r, p, y); q.normalize();
    geometry_msgs::msg::Quaternion out; out.x=q.x(); out.y=q.y(); out.z=q.z(); out.w=q.w();
    return out;
}

void WbqpControllerNode::publishArmTwist(const double J6x9_colmajor[54],
                                         const double qdot9[9]) const
{
    using Mat6x6 = Eigen::Matrix<double, 6, 6, Eigen::ColMajor>;
    using Vec6   = Eigen::Matrix<double, 6, 1>;

    // Map without copying: first 6 columns of J (col-major) are contiguous,
    // and the first 6 elements of qdot are the arm joints.
    const Eigen::Map<const Mat6x6> J_arm(J6x9_colmajor);  // J(:, 0..5)
    const Eigen::Map<const Vec6>   qdot_arm(qdot9);       // qdot(0..5)

    Vec6 v_arm;
    v_arm.noalias() = J_arm * qdot_arm;  // 6x6 * 6x1

    geometry_msgs::msg::Twist msg{};
    msg.linear.x  = v_arm(0);
    msg.linear.y  = v_arm(1);
    msg.linear.z  = v_arm(2);
    msg.angular.x = v_arm(3);
    msg.angular.y = v_arm(4);
    msg.angular.z = v_arm(5);

    pub_arm_vel_->publish(msg);
}

bool WbqpControllerNode::checkXoptValues(const double x_opt[9]) const
{
    bool ok {true};

    for (int i = 0; i < 9; ++i)
    {
        const double v = x_opt[i];

        if (std::isnan(v))
        {
            RCLCPP_ERROR(this->get_logger(), "x_opt[%d] is NaN", i);
            ok = false;
            continue;
        }

        if (std::isinf(v))
        {
            if (std::signbit(v))
            {
                RCLCPP_ERROR(this->get_logger(), "x_opt[%d] is -inf", i);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "x_opt[%d] is +inf", i);
            }
            ok = false;
            continue;
        }

        if (v > 1000.0)
        {
            RCLCPP_WARN(this->get_logger(), "x_opt[%d] = %.6f > +1000", i, v);
            ok = false;
        }
        else if (v < -1000.0)
        {
            RCLCPP_WARN(this->get_logger(), "x_opt[%d] = %.6f < -1000", i, v);
            ok = false;
        }
    }

    return ok;
}

// ------------------------------ SPINNER ----------------------------- //
void WbqpControllerNode::shutdown_handler()
{
    RCLCPP_INFO(get_logger(), "Whole Body Controller mean time: %f s", spinner_mean_);
}

void WbqpControllerNode::spinner()
{
    // Add the node to the executor
    executor_.add_node(shared_from_this());

    // Create a steady clock to measure time
	rclcpp::Clock steady_clock(RCL_STEADY_TIME);

    // Create timer callback with specified frequency (loop_rate_ in Hz)
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(dt_),  // period in seconds
        [this, &steady_clock]()
            {
                // This is the main loop for the node
                auto start_time = steady_clock.now();
                // Compute joints speed cmds
                bool step_done = loopStep();
                // Calculate the mean time for each iteration of the spinner
                double elapsed_time = (steady_clock.now() - start_time).seconds();
                spinner_mean_ = (spinner_mean_ * static_cast<double>(num_samples_) + elapsed_time) / static_cast<double>(num_samples_ + 1);
                num_samples_++;

                if (mode_ == Mode::DEBUG && step_done)
                {
                    RCLCPP_INFO(this->get_logger(),
                        "[DEBUG] iter=%llu dt=%.6f s mean=%.6f s",
                        num_samples_, elapsed_time, spinner_mean_);

                    dbg_.step_once = false;
                }
            });

    // Start spinning
    executor_.spin();

    // Shutdown the executor
    rclcpp::shutdown();
}

// --------------------------- DEBUG MENU -------------------------- //
void WbqpControllerNode::declare_and_setup_debug_params_()
{
    this->declare_parameter<std::string>("mode", "run");    // "run" | "debug"
    this->declare_parameter<bool>("console_menu", true);    // true => enable stdin menu

    // DEBUG input arrays (values in DEGREES where applicable)
    this->declare_parameter<std::vector<double>>("debug.P_N2B", {0,0,0});                   // meters
    this->declare_parameter<std::vector<double>>("debug.q_deg", {0,0,0,0,0,0});             // deg
    this->declare_parameter<std::vector<double>>("debug.theta_N2B_deg", {0,0,0});           // deg
    this->declare_parameter<std::vector<double>>("debug.theta_W2N_deg", {0,0,0});           // deg
    this->declare_parameter<std::vector<double>>("debug.u_star", {0,0,0,0,0,0});
    this->declare_parameter<std::vector<double>>("debug.dotq_prev", {0,0,0,0,0,0,0,0,0});
    this->declare_parameter<bool>("debug.step_once", false);

    // Initial read
    apply_params_();

    // Listen for changes
    param_cb_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params)
        {
            std::lock_guard<std::mutex> lk(dbg_mtx_);
            for (const auto &p : params)
            {
                if (p.get_name() == "mode")
                {
                    const auto v = p.as_string();
                    mode_ = (v == "debug") ? Mode::DEBUG : Mode::RUN;
                }
                else if (p.get_name() == "console_menu")
                {
                    console_menu_ = p.as_bool();
                    // Start/stop menu thread dynamically if you want (optional)
                }
                else if (p.get_name() == "debug.P_N2B")
                {
                    auto v = p.as_double_array();
                    if (v.size() == 3) { for (int i=0;i<3;++i) { dbg_.P_N2B[i] = v[i]; } }
                }
                else if (p.get_name() == "debug.q_deg")
                {
                    auto v = p.as_double_array();
                    if (v.size() == 6)
                    {
                        for (int i=0;i<6;++i) { dbg_.q_rad[i] = v[i] * M_PI / 180.0; }
                    }
                }
                else if (p.get_name() == "debug.theta_N2B_deg")
                {
                    auto v = p.as_double_array();
                    if (v.size() == 3) { for (int i=0;i<3;++i) { dbg_.theta_N2B_rad[i] = v[i] * M_PI / 180.0; } }
                }
                else if (p.get_name() == "debug.theta_W2N_deg")
                {
                    auto v = p.as_double_array();
                    if (v.size() == 3) { for (int i=0;i<3;++i) { dbg_.theta_W2N_rad[i] = v[i] * M_PI / 180.0; } }
                }
                else if (p.get_name() == "debug.u_star")
                {
                    auto v = p.as_double_array();
                    if (v.size() == 6) { for (int i=0;i<6;++i) { dbg_.u_star[i] = v[i]; } }
                }
                else if (p.get_name() == "debug.dotq_prev")
                {
                    auto v = p.as_double_array();
                    if (v.size() == 9) { for (int i=0;i<9;++i) { dbg_.dotq_prev[i] = v[i]; } }
                }
                else if (p.get_name() == "debug.step_once")
                {
                    dbg_.step_once = p.as_bool();
                }
            }
            rcl_interfaces::msg::SetParametersResult res;
            res.successful = true;
            return res;
        }
    );

    // Start menu thread if enabled
    if (console_menu_) {start_menu_thread_();}
}

void WbqpControllerNode::apply_params_()
{
    std::lock_guard<std::mutex> lk(dbg_mtx_);
    const auto mode_str = this->get_parameter("mode").as_string();
    mode_ = (mode_str == "debug") ? Mode::DEBUG : Mode::RUN;

    console_menu_ = this->get_parameter("console_menu").as_bool();

    auto P = this->get_parameter("debug.P_N2B").as_double_array();
    if (P.size() == 3) { for (int i=0;i<3;++i) { dbg_.P_N2B[i] = P[i]; } }

    auto qdeg = this->get_parameter("debug.q_deg").as_double_array();
    if (qdeg.size() == 6) { for (int i=0;i<6;++i) { dbg_.q_rad[i] = qdeg[i] * M_PI / 180.0; } }

    auto tNB = this->get_parameter("debug.theta_N2B_deg").as_double_array();
    if (tNB.size() == 3) { for (int i=0;i<3;++i) { dbg_.theta_N2B_rad[i] = tNB[i] * M_PI / 180.0; } }

    auto tWN = this->get_parameter("debug.theta_W2N_deg").as_double_array();
    if (tWN.size() == 3) { for (int i=0;i<3;++i) { dbg_.theta_W2N_rad[i] = tWN[i] * M_PI / 180.0; } }

    auto us = this->get_parameter("debug.u_star").as_double_array();
    if (us.size() == 6) { for (int i=0;i<6;++i) { dbg_.u_star[i] = us[i]; } }

    auto ws = this->get_parameter("debug.dotq_prev").as_double_array();
    if (ws.size() == 9) { for (int i=0;i<9;++i) { dbg_.dotq_prev[i] = ws[i]; } }

    dbg_.step_once = this->get_parameter("debug.step_once").as_bool();
}

void WbqpControllerNode::start_menu_thread_()
{
    if (!console_menu_) { return; }
    stop_menu_ = false;

    menu_thread_ = std::thread([this]()
    {
        auto deg2rad = [](double d) { return d * M_PI / 180.0; };

        while (!stop_menu_)
        {
            std::cout << "\n[WBQP DEBUG MENU]\n";
            std::cout << "1) Set q [deg x6]\n";
            std::cout << "2) Set theta_N2B [deg x3]\n";
            std::cout << "3) Set theta_W2N [deg x3]\n";
            std::cout << "4) Set P_N2B [m x3]\n";
            std::cout << "5) Set u_star [x6]\n";
            std::cout << "6) Set dotq_prev [x9]\n";
            std::cout << "7) STEP ONCE\n";
            std::cout << "8) Toggle mode (run/debug)\n";
            std::cout << "9) Quit menu\n> " << std::flush;

            int opt = 0;
            if (!(std::cin >> opt)) { std::cin.clear(); std::cin.ignore(1024, '\n'); continue; }

            if (opt == 9)
            {
                break;
            }

            std::lock_guard<std::mutex> lk(dbg_mtx_);

            auto read_vec = [](int n)
            {
                std::vector<double> v(n, 0.0);
                for (int i=0;i<n;++i) { std::cin >> v[i]; }
                return v;
            };

            if (opt == 1)
            {
                std::cout << "Enter 6 angles [deg]: ";
                auto v = read_vec(6);
                for (int i=0;i<6;++i) { dbg_.q_rad[i] = deg2rad(v[i]); }
            }
            else if (opt == 2)
            {
                std::cout << "Enter 3 angles [deg]: ";
                auto v = read_vec(3);
                for (int i=0;i<3;++i) { dbg_.theta_N2B_rad[i] = deg2rad(v[i]); }
            }
            else if (opt == 3)
            {
                std::cout << "Enter 3 angles [deg]: ";
                auto v = read_vec(3);
                for (int i=0;i<3;++i) { dbg_.theta_W2N_rad[i] = deg2rad(v[i]); }
            }
            else if (opt == 4)
            {
                std::cout << "Enter 3 positions [m]: ";
                auto v = read_vec(3);
                for (int i=0;i<3;++i) { dbg_.P_N2B[i] = v[i]; }
            }
            else if (opt == 5)
            {
                std::cout << "Enter 6 values: ";
                auto v = read_vec(6);
                for (int i=0;i<6;++i) { dbg_.u_star[i] = v[i]; }
            }
            else if (opt == 6)
            {
                std::cout << "Enter 9 values: ";
                auto v = read_vec(9);
                for (int i=0;i<9;++i) { dbg_.dotq_prev[i] = v[i]; }
            }
            else if (opt == 7)
            {
                dbg_.step_once = true;
            }
            else if (opt == 8)
            {
                if (mode_ == Mode::RUN) { mode_ = Mode::DEBUG; }
                else { mode_ = Mode::RUN; }
                std::cout << "Mode is now: " << ((mode_==Mode::DEBUG) ? "DEBUG" : "RUN") << "\n";
            }

            // print a quick summary after each action
            std::cout << "OK. (Values updated.)\n";
        }

        std::cout << "[WBQP MENU] exiting.\n";
    });
}

void WbqpControllerNode::stop_menu_thread_()
{
    if (!console_menu_) { return; }
    stop_menu_ = true;
    if (menu_thread_.joinable()) { menu_thread_.join(); }
}

// -------------------------- MAIN FUNCTION -------------------------- //
int main(int argc, char ** argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create a node using the rclcpp library
    auto node = std::make_shared<WbqpControllerNode>();

	// Spin the node to start handling callbacks and timers
    node->spinner();

    return 0;
}
