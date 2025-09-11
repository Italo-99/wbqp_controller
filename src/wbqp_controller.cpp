#include "wbqp_controller/wbqp_controller.hpp"

using std::placeholders::_1;

WbqpControllerNode::WbqpControllerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("wbqp_controller", options) {
    // --- Params ---
    topic_joint_state_ = this->declare_parameter<std::string>("topics.joint_state", "/joint_states");
    topic_twist_cmd_   = this->declare_parameter<std::string>("topics.twist_cmd",   "/ee_twist_cmd");
    topic_cmd_vel_     = this->declare_parameter<std::string>("topics.cmd_vel",     "/cmd_vel");
    topic_q_speed_     = this->declare_parameter<std::string>("topics.q_speed",     "/manipulator/js_cmd_vel");
    dt_                = this->declare_parameter<double>("control.dt", 0.02);

    auto cols_param = this->declare_parameter<std::vector<int64_t>>("qp.cols", {1,2,3,4,5,6,7,8,12});
    cols1based_.assign(cols_param.begin(), cols_param.end());

    auto p_n2b = this->declare_parameter<std::vector<double>>("kinematics.P_N2B", {0.187, 0.0, 0.22});
    auto th_n2b = this->declare_parameter<std::vector<double>>("kinematics.theta_N2B", {0.0, 0.0, 0.0});
    for (int i=0;i<3;++i){ P_N2B_[i] = p_n2b[i]; theta_N2B_[i] = th_n2b[i]; }

    map_frame_       = this->declare_parameter<std::string>("frames.map", "map");
    base_frame_      = this->declare_parameter<std::string>("frames.base", "base");
    base_link_frame_ = this->declare_parameter<std::string>("frames.base_link", "base_link");

    // Optional MATLAB init calls
    #ifdef HAS_WBJ_INIT
      WholeBodyJacobian_initialize();
    #endif
    #ifdef HAS_WBQP_INIT_INIT
      wbqp_init_initialize();
    #endif
    #ifdef HAS_WBQP_SOLVE_INIT
      wbqp_solve_initialize();
    #endif

    // Build cfg and run wbqp_init once
    struct0_T cfg{};
    fillCfgStruct(cfg);
    wbqp_init(&cfg, &qp_);
    qp_initialized_ = true;

    // ROS I/O
    sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
        topic_joint_state_, rclcpp::SensorDataQoS(),
        std::bind(&WbqpControllerNode::jointStateCb, this, _1));

    sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
        topic_twist_cmd_, 10,
        std::bind(&WbqpControllerNode::twistCmdCb, this, _1));

    pub_cmd_vel_  = this->create_publisher<geometry_msgs::msg::Twist>(topic_cmd_vel_, 10);
    pub_q_speed_  = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic_q_speed_, 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    pub_base_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/base_pose", 10);

    // Publish static TF base -> base_link once
    publishStaticBaseToBaseLink();

    // Loop timer
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(dt_),
        std::bind(&WbqpControllerNode::loopStep, this));
}

WbqpControllerNode::~WbqpControllerNode() {
    // Optional MATLAB terminate calls
    #ifdef HAS_WBQP_SOLVE_TERM
      wbqp_solve_terminate();
    #endif
    #ifdef HAS_WBQP_INIT_TERM
      wbqp_init_terminate();
    #endif
    #ifdef HAS_WBJ_TERM
      WholeBodyJacobian_terminate();
    #endif
}

void WbqpControllerNode::jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.size() >= 6) {
        for (int i=0;i<6;++i) { q_pos_[i] = msg->position[i]; }
    }
    if (msg->velocity.size() >= 6) {
        for (int i=0;i<6;++i) { q_vel_[i] = msg->velocity[i]; }
    }
    have_js_ = true;
}

void WbqpControllerNode::twistCmdCb(const geometry_msgs::msg::Twist::SharedPtr msg) {
    u_star_[0]=msg->linear.x;  u_star_[1]=msg->linear.y;  u_star_[2]=msg->linear.z;
    u_star_[3]=msg->angular.x; u_star_[4]=msg->angular.y; u_star_[5]=msg->angular.z;
    have_twist_ = true;
}

void WbqpControllerNode::loopStep() {
    if (!qp_initialized_ || !have_js_ || !have_twist_) { return; }

    // 1) Build WBJ input and compute 6x12 Jacobian (column-major)
    double in1[15];
    buildIn15(in1);
    double A6x12_colmajor[72];
    WholeBodyJacobian(in1, A6x12_colmajor);

    // 2) Fill in-struct for wbqp_solve
    struct1_T in{};
    fillInputStruct(in, A6x12_colmajor);

    // 3) Solve QP
    double x_opt[9];
    struct2_T dbg{};
    wbqp_solve(reinterpret_cast<const struct0_T *>(&qp_), &in, x_opt, &dbg);

    // Warm start
    for (int i=0;i<9;++i) { dotq_prev_[i] = x_opt[i]; }

    // Integrate base and publish TF/Pose
    integrateAndPublishBase(x_opt);

    // Publish outputs
    publishOutputs(x_opt);
}

void WbqpControllerNode::buildIn15(double out_in1_15[15]) const {
    out_in1_15[0]=P_N2B_[0]; out_in1_15[1]=P_N2B_[1]; out_in1_15[2]=P_N2B_[2];
    for (int i=0;i<6;++i){ out_in1_15[3+i] = q_pos_[i]; }
    out_in1_15[9]=theta_N2B_[0]; out_in1_15[10]=theta_N2B_[1]; out_in1_15[11]=theta_N2B_[2];
    out_in1_15[12]=theta_W2N_[0]; out_in1_15[13]=theta_W2N_[1]; out_in1_15[14]=theta_W2N_[2];
}

void WbqpControllerNode::reduce_J_6x12_to_6x9(const double J6x12_colmajor[72],
                                              const std::vector<int> &cols1based,
                                              double J6x9_colmajor[54]) {
    for (int j=0;j<9;++j) {
        int src_c = cols1based[j] - 1;
        for (int r=0;r<6;++r) {
            J6x9_colmajor[r + 6*j] = J6x12_colmajor[r + 6*src_c];
        }
    }
}

void WbqpControllerNode::fillCfgStruct(struct0_T &cfg) {
    std::memset(&cfg, 0, sizeof(cfg));

    // cols
    for (int i=0;i<9 && i<(int)cols1based_.size(); ++i) { cfg.cols[i] = static_cast<int>(cols1based_[i]); }

    // Pull scalar params (with defaults from declare_parameter)
    cfg.beta_arm   = this->get_parameter("qp.beta_arm").get_value<double>();
    cfg.alpha_xy   = this->get_parameter("qp.alpha_xy").get_value<double>();
    cfg.alpha_yaw  = this->get_parameter("qp.alpha_yaw").get_value<double>();
    cfg.w_lin      = this->get_parameter("qp.w_lin").get_value<double>();
    cfg.w_ang      = this->get_parameter("qp.w_ang").get_value<double>();
    cfg.nu         = this->get_parameter("qp.nu").get_value<double>();
    cfg.max_dotq   = this->get_parameter("qp.max_dotq").get_value<double>();
    cfg.max_V      = this->get_parameter("qp.max_V").get_value<double>();
    cfg.max_Omegaz = this->get_parameter("qp.max_Omegaz").get_value<double>();

    auto qmin = this->get_parameter("limits.qmin").as_double_array();
    auto qmax = this->get_parameter("limits.qmax").as_double_array();
    for (int i=0;i<6;++i) { cfg.qmin[i] = qmin[i]; cfg.qmax[i] = qmax[i]; }

    cfg.qddot_max  = this->get_parameter("limits.qddot_max").get_value<double>();
    cfg.a_lin_max  = this->get_parameter("limits.a_lin_max").get_value<double>();
    cfg.alpha_max  = this->get_parameter("limits.alpha_max").get_value<double>();
}

void WbqpControllerNode::fillInputStruct(struct1_T &in, const double J6x12_colmajor[72]) const {
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

void WbqpControllerNode::publishOutputs(const double x_opt[9]) {
    geometry_msgs::msg::Twist base;
    base.linear.x  = x_opt[6];
    base.linear.y  = x_opt[7];
    base.angular.z = x_opt[8];
    pub_cmd_vel_->publish(base);

    std_msgs::msg::Float64MultiArray qspd;
    qspd.data.resize(6);
    for (int i=0;i<6;++i) { qspd.data[i] = x_opt[i]; }
    pub_q_speed_->publish(qspd);
}

geometry_msgs::msg::Quaternion WbqpControllerNode::quatFromRPY(double r, double p, double y) {
    tf2::Quaternion q; q.setRPY(r, p, y);
    geometry_msgs::msg::Quaternion out; out.x=q.x(); out.y=q.y(); out.z=q.z(); out.w=q.w();
    return out;
}

void WbqpControllerNode::publishStaticBaseToBaseLink() {
    geometry_msgs::msg::TransformStamped st;
    st.header.stamp = this->get_clock()->now();
    st.header.frame_id = base_frame_;
    st.child_frame_id  = base_link_frame_;
    st.transform.translation.x = P_N2B_[0];
    st.transform.translation.y = P_N2B_[1];
    st.transform.translation.z = P_N2B_[2];
    st.transform.rotation = quatFromRPY(theta_N2B_[0], theta_N2B_[1], theta_N2B_[2]);
    static_tf_broadcaster_->sendTransform(st);
}

void WbqpControllerNode::integrateAndPublishBase(const double x_opt[9]) {
    const double Vx_b = x_opt[6];
    const double Vy_b = x_opt[7];
    const double wz   = x_opt[8];
    const double th   = theta_W2N_[2];
    const double c    = std::cos(th);
    const double s    = std::sin(th);

    const double vx_w = c * Vx_b - s * Vy_b;
    const double vy_w = s * Vx_b + c * Vy_b;

    x_base_      += vx_w * dt_;
    y_base_      += vy_w * dt_;
    theta_W2N_[2] += wz   * dt_;

    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = this->get_clock()->now();
    ps.header.frame_id = map_frame_;
    ps.pose.position.x = x_base_;
    ps.pose.position.y = y_base_;
    ps.pose.position.z = 0.0;
    ps.pose.orientation = quatFromRPY(0.0, 0.0, theta_W2N_[2]);
    pub_base_pose_->publish(ps);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = ps.header.stamp;
    tf.header.frame_id = map_frame_;
    tf.child_frame_id  = base_frame_;
    tf.transform.translation.x = x_base_;
    tf.transform.translation.y = y_base_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = ps.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WbqpControllerNode>());
    rclcpp::shutdown();
    return 0;
}
