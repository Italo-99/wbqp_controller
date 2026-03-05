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

#include <algorithm>
#include <cctype>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <unordered_map>

#include <Eigen/Geometry>
#include <tp_control/tasks/joint_limit_task.hpp>
#include <tp_control/tasks/pointcloud_collision_task.hpp>
#include <tp_control/tasks/singularity_task.hpp>
#include <tp_control/tasks/tracking_task.hpp>

namespace {

constexpr int kIdBaseCorner0 = 0;
constexpr int kIdBaseCorner1 = 1;
constexpr int kIdBaseCorner2 = 2;
constexpr int kIdBaseCorner3 = 3;
constexpr int kIdJointEndpoint1 = 10;   // ... +5 => joint6 endpoint
constexpr int kIdTcp = 20;
constexpr int kIdBaseLinkOrigin = 30;
constexpr int kLinkSampleBase = 100;    // id = 100 + 100*link_idx(1..6) + sample_idx(1..N)

int makeLinkSampleId(int link_idx_1based, int sample_idx_1based)
{
    return kLinkSampleBase + (100 * link_idx_1based) + sample_idx_1based;
}

Eigen::Matrix3d rotX(double rx)
{
    const double c = std::cos(rx);
    const double s = std::sin(rx);
    Eigen::Matrix3d R;
    R << 1.0, 0.0, 0.0,
         0.0,   c,  -s,
         0.0,   s,   c;
    return R;
}

Eigen::Matrix3d rotY(double ry)
{
    const double c = std::cos(ry);
    const double s = std::sin(ry);
    Eigen::Matrix3d R;
    R <<   c, 0.0,   s,
         0.0, 1.0, 0.0,
          -s, 0.0,   c;
    return R;
}

Eigen::Matrix3d rotZ(double rz)
{
    const double c = std::cos(rz);
    const double s = std::sin(rz);
    Eigen::Matrix3d R;
    R <<   c,  -s, 0.0,
           s,   c, 0.0,
         0.0, 0.0, 1.0;
    return R;
}

Eigen::Matrix3d rpyToR(double r, double p, double y)
{
    const double cr = std::cos(r), sr = std::sin(r);
    const double cp = std::cos(p), sp = std::sin(p);
    const double cy = std::cos(y), sy = std::sin(y);

    Eigen::Matrix3d R;
    R << cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr,
         sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr,
           -sp,        cp*sr,              cp*cr;
    return R;
}

// Same planar mapping convention already used in base integration:
// [vx_w; vy_w] = [ c s; -s c ] [vx_b; vy_b]
Eigen::Matrix3d rotationWN(double yaw)
{
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R(0,0) = c;  R(0,1) = s;
    R(1,0) = -s; R(1,1) = c;
    return R;
}

tp_control::Pose6D poseMsgToTpPose(const geometry_msgs::msg::Pose& msg)
{
    tp_control::Pose6D out;
    out.p << msg.position.x, msg.position.y, msg.position.z;

    Eigen::Quaterniond q(msg.orientation.w,
                         msg.orientation.x,
                         msg.orientation.y,
                         msg.orientation.z);
    if (q.norm() < 1.0e-12)
    {
        out.R = Eigen::Matrix3d::Identity();
    }
    else
    {
        out.R = q.normalized().toRotationMatrix();
    }
    return out;
}

Eigen::Matrix3d integrateRotationExpMap(const Eigen::Matrix3d& R,
                                        const Eigen::Vector3d& omega,
                                        double dt)
{
    const double angle = omega.norm() * dt;
    if (angle < 1.0e-12)
    {
        return R;
    }
    const Eigen::Vector3d axis = omega.normalized();
    return R * Eigen::AngleAxisd(angle, axis).toRotationMatrix();
}

Eigen::Matrix4d dhTransform(const MobileManipulatorKinematics::DHParam& dh, double q)
{
    const double cq = std::cos(q);
    const double sq = std::sin(q);
    const double ca = std::cos(dh.alpha);
    const double sa = std::sin(dh.alpha);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0,0) = cq;        T(0,1) = -sq * ca;   T(0,2) =  sq * sa;   T(0,3) = dh.a * cq;
    T(1,0) = sq;        T(1,1) =  cq * ca;   T(1,2) = -cq * sa;   T(1,3) = dh.a * sq;
    T(2,0) = 0.0;       T(2,1) =  sa;        T(2,2) =  ca;        T(2,3) = dh.d;
    return T;
}

Eigen::Matrix4d tcpOffsetTransform(const std::array<double, 6>& tcp_offset)
{
    const Eigen::Matrix3d R = rotZ(tcp_offset[5]) * rotY(tcp_offset[4]) * rotX(tcp_offset[3]);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = Eigen::Vector3d(tcp_offset[0], tcp_offset[1], tcp_offset[2]);
    return T;
}

struct CollisionPointModelConfig
{
    std::array<MobileManipulatorKinematics::DHParam, 6> dh_params{};
    std::array<double, 6> tcp_offset{};
    Eigen::Vector3d p_NB = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_NB = Eigen::Matrix3d::Identity();
    double base_size_x = 0.70;
    double base_size_y = 0.50;
    double base_delta_h = 0.05;
    int samples_per_link = 0;
    bool include_joint_endpoints = true;
    bool include_tcp = true;
};

struct RobotPointState
{
    Eigen::Vector3d p_world = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 9> J_world_legacy = Eigen::Matrix<double, 3, 9>::Zero();
};

struct RobotPointModelCache
{
    std::unordered_map<int, RobotPointState> points;
    std::vector<int> collision_point_ids;
    Eigen::Vector3d ez_B_world = Eigen::Vector3d::UnitZ();
    double tcp_height_B = 0.0;
    Eigen::Matrix<double, 1, 9> tcp_height_row_legacy = Eigen::Matrix<double, 1, 9>::Zero();
};

Eigen::Matrix<double, 3, 9> reorderLegacyToTp(const Eigen::Matrix<double, 3, 9>& J_legacy)
{
    Eigen::Matrix<double, 3, 9> J_tp = Eigen::Matrix<double, 3, 9>::Zero();
    J_tp.col(0) = J_legacy.col(6);
    J_tp.col(1) = J_legacy.col(7);
    J_tp.col(2) = J_legacy.col(8);
    for (int i = 0; i < 6; ++i) { J_tp.col(i + 3) = J_legacy.col(i); }
    return J_tp;
}

std::vector<int> buildCollisionPointIds(const CollisionPointModelConfig& cfg)
{
    std::vector<int> ids;
    ids.reserve(4 + 6 + 1 + (6 * std::max(0, cfg.samples_per_link)));

    ids.push_back(kIdBaseCorner0);
    ids.push_back(kIdBaseCorner1);
    ids.push_back(kIdBaseCorner2);
    ids.push_back(kIdBaseCorner3);

    if (cfg.include_joint_endpoints)
    {
        for (int i = 0; i < 6; ++i)
        {
            ids.push_back(kIdJointEndpoint1 + i);
        }
    }

    const double tcp_translation_norm =
        std::abs(cfg.tcp_offset[0]) + std::abs(cfg.tcp_offset[1]) + std::abs(cfg.tcp_offset[2]);
    if (cfg.include_tcp && tcp_translation_norm > 1.0e-12)
    {
        ids.push_back(kIdTcp);
    }

    if (cfg.samples_per_link > 0)
    {
        for (int link_idx = 1; link_idx <= 6; ++link_idx)
        {
            for (int s = 1; s <= cfg.samples_per_link; ++s)
            {
                ids.push_back(makeLinkSampleId(link_idx, s));
            }
        }
    }

    return ids;
}

void computePointJacobianN(
    const std::array<Eigen::Vector3d, 7>& origins_N,
    const std::array<Eigen::Vector3d, 6>& z_axes_N,
    const Eigen::Vector3d& p_N,
    int last_joint_idx,
    Eigen::Matrix<double, 3, 9>& J_N)
{
    J_N.setZero();

    for (int j = 0; j < 6; ++j)
    {
        if (j <= last_joint_idx)
        {
            J_N.col(j) = z_axes_N[j].cross(p_N - origins_N[j]);
        }
    }

    J_N.col(6) = Eigen::Vector3d::UnitX();
    J_N.col(7) = Eigen::Vector3d::UnitY();
    J_N.col(8) = Eigen::Vector3d::UnitZ().cross(p_N);
}

void addPointWorld(
    RobotPointModelCache& cache,
    int id,
    const Eigen::Vector3d& p_N,
    int last_joint_idx,
    const std::array<Eigen::Vector3d, 7>& origins_N,
    const std::array<Eigen::Vector3d, 6>& z_axes_N,
    const Eigen::Matrix3d& R_WN,
    const Eigen::Vector3d& p_WN)
{
    Eigen::Matrix<double, 3, 9> J_N = Eigen::Matrix<double, 3, 9>::Zero();
    computePointJacobianN(origins_N, z_axes_N, p_N, last_joint_idx, J_N);

    RobotPointState st;
    st.p_world = p_WN + R_WN * p_N;
    st.J_world_legacy = R_WN * J_N;
    cache.points[id] = st;
}

RobotPointModelCache computeRobotPointModel(
    const CollisionPointModelConfig& cfg,
    double x_base,
    double y_base,
    double yaw,
    const std::array<double, 6>& q_arm)
{
    RobotPointModelCache cache;
    cache.collision_point_ids = buildCollisionPointIds(cfg);

    const Eigen::Matrix3d R_WN = rotationWN(yaw);
    const Eigen::Vector3d p_WN(x_base, y_base, 0.0);

    std::array<Eigen::Vector3d, 7> origins_B{};
    std::array<Eigen::Vector3d, 7> origins_N{};
    std::array<Eigen::Vector3d, 6> z_axes_B{};
    std::array<Eigen::Vector3d, 6> z_axes_N{};

    origins_B[0] = Eigen::Vector3d::Zero();
    z_axes_B[0] = Eigen::Vector3d::UnitZ();

    Eigen::Matrix4d T_Bi = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 6; ++i)
    {
        T_Bi = T_Bi * dhTransform(cfg.dh_params[i], q_arm[i]);
        origins_B[i + 1] = T_Bi.block<3,1>(0,3);
        if (i < 5)
        {
            z_axes_B[i + 1] = T_Bi.block<3,3>(0,0) * Eigen::Vector3d::UnitZ();
        }
    }

    for (int i = 0; i < 7; ++i)
    {
        origins_N[i] = cfg.R_NB * origins_B[i] + cfg.p_NB;
        if (i < 6) { z_axes_N[i] = cfg.R_NB * z_axes_B[i]; }
    }

    const Eigen::Matrix4d T_BC = T_Bi * tcpOffsetTransform(cfg.tcp_offset);
    const Eigen::Vector3d p_B_tcp = T_BC.block<3,1>(0,3);
    const Eigen::Vector3d p_N_tcp = cfg.R_NB * p_B_tcp + cfg.p_NB;

    // Base link origin in world (needed for TCP-height guard, always available).
    addPointWorld(cache, kIdBaseLinkOrigin, cfg.p_NB, -1, origins_N, z_axes_N, R_WN, p_WN);

    // Base corners from mobile-base footprint reconstructed from P_N2B.
    const Eigen::Vector3d p_B_center = -cfg.R_NB.transpose() * cfg.p_NB;
    const double hx = 0.5 * cfg.base_size_x;
    const double hy = 0.5 * cfg.base_size_y;
    const std::array<Eigen::Vector2d, 4> corner_xy_B{
        Eigen::Vector2d(p_B_center.x() + hx, p_B_center.y() + hy),
        Eigen::Vector2d(p_B_center.x() + hx, p_B_center.y() - hy),
        Eigen::Vector2d(p_B_center.x() - hx, p_B_center.y() + hy),
        Eigen::Vector2d(p_B_center.x() - hx, p_B_center.y() - hy)
    };
    for (int i = 0; i < 4; ++i)
    {
        const Eigen::Vector3d p_B(corner_xy_B[i].x(), corner_xy_B[i].y(), -cfg.base_delta_h);
        const Eigen::Vector3d p_N = cfg.R_NB * p_B + cfg.p_NB;
        addPointWorld(cache, kIdBaseCorner0 + i, p_N, -1, origins_N, z_axes_N, R_WN, p_WN);
    }

    // Joint endpoints O1..O6.
    if (cfg.include_joint_endpoints)
    {
        for (int i = 1; i <= 6; ++i)
        {
            addPointWorld(cache, kIdJointEndpoint1 + (i - 1), origins_N[i], i - 1,
                          origins_N, z_axes_N, R_WN, p_WN);
        }
    }

    // Optional link samples between consecutive joint endpoints.
    if (cfg.samples_per_link > 0)
    {
        for (int link_idx = 1; link_idx <= 6; ++link_idx)
        {
            const Eigen::Vector3d p0 = origins_N[link_idx - 1];
            const Eigen::Vector3d p1 = origins_N[link_idx];
            for (int s = 1; s <= cfg.samples_per_link; ++s)
            {
                const double t = static_cast<double>(s) / static_cast<double>(cfg.samples_per_link + 1);
                const Eigen::Vector3d p = (1.0 - t) * p0 + t * p1;
                addPointWorld(cache, makeLinkSampleId(link_idx, s), p, link_idx - 1,
                              origins_N, z_axes_N, R_WN, p_WN);
            }
        }
    }

    // TCP point is always computed (guard needs it); collision list may choose to use it or not.
    addPointWorld(cache, kIdTcp, p_N_tcp, 5, origins_N, z_axes_N, R_WN, p_WN);

    // TCP height in arm-base frame B: z_B = ez_B^T (p_tcp^W - p_B^W)
    cache.ez_B_world = R_WN * cfg.R_NB.col(2);
    const auto it_tcp = cache.points.find(kIdTcp);
    const auto it_B = cache.points.find(kIdBaseLinkOrigin);
    if (it_tcp != cache.points.end() && it_B != cache.points.end())
    {
        cache.tcp_height_B = cache.ez_B_world.dot(it_tcp->second.p_world - it_B->second.p_world);
        const Eigen::Matrix<double, 3, 9> Jrel = it_tcp->second.J_world_legacy - it_B->second.J_world_legacy;
        cache.tcp_height_row_legacy = cache.ez_B_world.transpose() * Jrel;
    }

    return cache;
}

class TcpHeightTask final : public tp_control::ITask
{
public:
    struct Params
    {
        int priority = 3;
        double z_min = 0.03;
        double z_act = 0.06;
        double k = 2.0;
        int tcp_point_id = kIdTcp;
        int base_link_point_id = kIdBaseLinkOrigin;
        Eigen::Matrix3d R_NB = Eigen::Matrix3d::Identity();
    };

    explicit TcpHeightTask(const Params& p) : p_(p) {}

    const char* name() const override { return "TcpHeightTask"; }
    int priority() const override { return p_.priority; }
    int dim() const override { return 1; }

    void configure(int n_dof) override
    {
        n_ = n_dof;
        p_.z_act = std::max(p_.z_act, p_.z_min);
    }

    void update(const Context& cx, Eigen::Ref<tp_control::Mat> J_out,
                Eigen::Ref<tp_control::Vec> d_out, Eigen::Ref<tp_control::Vec> a_out) override
    {
        J_out.setZero();
        d_out.setZero();
        a_out.setZero();

        if (cx.state.q.size() < 3)
        {
            return;
        }

        tp_control::Mat J_tcp(3, n_);
        tp_control::Mat J_base(3, n_);
        const auto p_tcp = cx.kin.pointPosition(tp_control::PointRequest{p_.tcp_point_id});
        const auto p_base = cx.kin.pointPosition(tp_control::PointRequest{p_.base_link_point_id});
        cx.kin.pointJacobian(tp_control::PointRequest{p_.tcp_point_id}, J_tcp);
        cx.kin.pointJacobian(tp_control::PointRequest{p_.base_link_point_id}, J_base);

        const double yaw = cx.state.q(2);
        const Eigen::Vector3d ez_B_world = rotationWN(yaw) * p_.R_NB.col(2);
        const double z_B = ez_B_world.dot(p_tcp - p_base);

        const Eigen::RowVectorXd J_row = ez_B_world.transpose() * (J_tcp - J_base);
        J_out.row(0) = J_row;

        // Smooth activation: fully active below z_min, inactive above z_act.
        double alpha = 0.0;
        if (z_B <= p_.z_min)
        {
            alpha = 1.0;
        }
        else if (z_B >= p_.z_act)
        {
            alpha = 0.0;
        }
        else
        {
            const double t = (p_.z_act - z_B) / std::max(1.0e-12, (p_.z_act - p_.z_min));
            const double tc = std::clamp(t, 0.0, 1.0);
            alpha = tc * tc * (3.0 - 2.0 * tc);
        }

        d_out(0) = p_.k * (p_.z_min - z_B);
        a_out(0) = alpha;
    }

private:
    Params p_;
    int n_ = 0;
};

class WbqpTpKinematicsProvider final : public tp_control::IKinematicsProvider
{
public:
    explicit WbqpTpKinematicsProvider(const MobileManipulatorKinematics* kin,
                                      bool jac_to_world,
                                      bool use_vendor_jacobian,
                                      const std::array<double, 3>& p_n2b,
                                      const std::array<double, 3>& theta_n2b,
                                      const std::vector<int>& cols1based,
                                      const CollisionPointModelConfig& collision_cfg)
    : kin_(kin),
      jac_to_world_(jac_to_world),
      use_vendor_jacobian_(use_vendor_jacobian),
      p_n2b_(p_n2b),
      theta_n2b_(theta_n2b),
      cols1based_(cols1based),
      collision_cfg_(collision_cfg)
    {
        if (kin_ == nullptr)
        {
            throw std::invalid_argument("WbqpTpKinematicsProvider: kin is null");
        }
        if (cols1based_.size() != 9)
        {
            throw std::invalid_argument("WbqpTpKinematicsProvider: cols1based must have size 9");
        }
        q_tp_.setZero(9);
    }

    void update(const tp_control::Vec& q) const override
    {
        if (q.size() != 9)
        {
            throw std::invalid_argument("WbqpTpKinematicsProvider::update expects q size 9");
        }
        q_tp_ = q;

        std::array<double, 6> q_arm{};
        for (int i = 0; i < 6; ++i) { q_arm[i] = q_tp_(i + 3); }
        point_cache_ = computeRobotPointModel(collision_cfg_, q_tp_(0), q_tp_(1), q_tp_(2), q_arm);
    }

    tp_control::Pose6D eePose() const override
    {
        const Eigen::Matrix<double, 6, 1> q_arm = q_tp_.segment<6>(3);
        Eigen::Matrix4d T;
        if (use_vendor_jacobian_)
        {
            // Keep TP pose/jacobian frame coherent in vendor branch.
            T = kin_->forwardKinematicsNC(q_arm);
        }
        else if (jac_to_world_)
        {
            MobileManipulatorKinematics::BasePose2D base_pose;
            base_pose.px = q_tp_(0);
            base_pose.py = q_tp_(1);
            base_pose.yaw = q_tp_(2);
            T = kin_->forwardKinematicsWC(q_arm, base_pose);
        }
        else
        {
            T = kin_->forwardKinematicsNC(q_arm);
        }

        tp_control::Pose6D out;
        out.p = T.block<3,1>(0,3);
        out.R = T.block<3,3>(0,0);
        return out;
    }

    void eeJacobian(Eigen::Ref<tp_control::Mat> J_out) const override
    {
        if (J_out.rows() != 6 || J_out.cols() != 9)
        {
            throw std::invalid_argument("WbqpTpKinematicsProvider::eeJacobian expects 6x9");
        }

        Eigen::Matrix<double, 6, 9> J_arm_base = Eigen::Matrix<double, 6, 9>::Zero();
        if (use_vendor_jacobian_)
        {
            double in1[15];
            in1[0] = p_n2b_[0];
            in1[1] = p_n2b_[1];
            in1[2] = p_n2b_[2];
            for (int i = 0; i < 6; ++i) { in1[3 + i] = q_tp_(i + 3); }
            in1[9] = theta_n2b_[0];
            in1[10] = theta_n2b_[1];
            in1[11] = theta_n2b_[2];
            in1[12] = 0.0;
            in1[13] = 0.0;
            in1[14] = q_tp_(2);

            double A6x12_colmajor[72];
            WholeBodyJacobian(in1, A6x12_colmajor);
            for (int j = 0; j < 9; ++j)
            {
                const int src_c = cols1based_[j] - 1;
                for (int r = 0; r < 6; ++r)
                {
                    J_arm_base(r, j) = A6x12_colmajor[r + 6 * src_c];
                }
            }
        }
        else
        {
            const Eigen::Matrix<double, 6, 1> q_arm = q_tp_.segment<6>(3);
            if (jac_to_world_)
            {
                MobileManipulatorKinematics::BasePose2D base_pose;
                base_pose.px = q_tp_(0);
                base_pose.py = q_tp_(1);
                base_pose.yaw = q_tp_(2);
                J_arm_base = kin_->jacobianWorld(q_arm, base_pose);
            }
            else
            {
                J_arm_base = kin_->jacobianBody(q_arm);
            }
        }

        // TP state ordering is [base(3), arm(6)].
        J_out.leftCols(3) = J_arm_base.rightCols(3); // base columns
        J_out.rightCols(6) = J_arm_base.leftCols(6); // arm columns
    }

    void armJacobian(Eigen::Ref<tp_control::Mat> J_arm_out) const override
    {
        if (J_arm_out.rows() != 6 || J_arm_out.cols() != 6)
        {
            throw std::invalid_argument("WbqpTpKinematicsProvider::armJacobian expects 6x6");
        }

        Eigen::Matrix<double, 6, 9> J_arm_base = Eigen::Matrix<double, 6, 9>::Zero();
        if (use_vendor_jacobian_)
        {
            double in1[15];
            in1[0] = p_n2b_[0];
            in1[1] = p_n2b_[1];
            in1[2] = p_n2b_[2];
            for (int i = 0; i < 6; ++i) { in1[3 + i] = q_tp_(i + 3); }
            in1[9] = theta_n2b_[0];
            in1[10] = theta_n2b_[1];
            in1[11] = theta_n2b_[2];
            in1[12] = 0.0;
            in1[13] = 0.0;
            in1[14] = q_tp_(2);

            double A6x12_colmajor[72];
            WholeBodyJacobian(in1, A6x12_colmajor);
            for (int j = 0; j < 9; ++j)
            {
                const int src_c = cols1based_[j] - 1;
                for (int r = 0; r < 6; ++r)
                {
                    J_arm_base(r, j) = A6x12_colmajor[r + 6 * src_c];
                }
            }
        }
        else
        {
            const Eigen::Matrix<double, 6, 1> q_arm = q_tp_.segment<6>(3);
            if (jac_to_world_)
            {
                MobileManipulatorKinematics::BasePose2D base_pose;
                base_pose.px = q_tp_(0);
                base_pose.py = q_tp_(1);
                base_pose.yaw = q_tp_(2);
                J_arm_base = kin_->jacobianWorld(q_arm, base_pose);
            }
            else
            {
                J_arm_base = kin_->jacobianBody(q_arm);
            }
        }

        J_arm_out = J_arm_base.leftCols(6);
    }

    void embedArmGradient(const Eigen::VectorXd& grad_arm, Eigen::Ref<tp_control::Vec> grad_full_out) const override
    {
        if (grad_arm.size() != 6 || grad_full_out.size() != 9)
        {
            throw std::invalid_argument("WbqpTpKinematicsProvider::embedArmGradient wrong size");
        }
        grad_full_out.setZero();
        grad_full_out.tail(6) = grad_arm;
    }

    Eigen::Vector3d pointPosition(const tp_control::PointRequest& req) const override
    {
        const auto it = point_cache_.points.find(req.id);
        if (it != point_cache_.points.end())
        {
            return it->second.p_world;
        }
        return Eigen::Vector3d::Zero();
    }

    void pointJacobian(const tp_control::PointRequest& req, Eigen::Ref<tp_control::Mat> Jp_out) const override
    {
        if (Jp_out.rows() != 3 || Jp_out.cols() != 9)
        {
            throw std::invalid_argument("WbqpTpKinematicsProvider::pointJacobian expects 3x9");
        }

        Jp_out.setZero();
        const auto it = point_cache_.points.find(req.id);
        if (it != point_cache_.points.end())
        {
            Jp_out = reorderLegacyToTp(it->second.J_world_legacy);
            return;
        }
    }

    int armDofs() const override { return 6; }

private:
    const MobileManipulatorKinematics* kin_ = nullptr;
    bool jac_to_world_ = false;
    bool use_vendor_jacobian_ = false;
    std::array<double, 3> p_n2b_{0.0, 0.0, 0.0};
    std::array<double, 3> theta_n2b_{0.0, 0.0, 0.0};
    std::vector<int> cols1based_{};
    CollisionPointModelConfig collision_cfg_{};
    mutable tp_control::Vec q_tp_;
    mutable RobotPointModelCache point_cache_;
};

} // namespace

// ------------------- CONSTRUCTOR & DESTRUCTOR ------------------- //
WbqpControllerNode::WbqpControllerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("wbqp_controller", options)
{
    // --- Params ---
    check_params();

    // --- Init C++ kinematics (wb_jac) if enabled ---
    initKinematics();

    // Build cfg and run wbqp_init once (QP mode only)
    struct10_T cfg{};
    fillCfgStruct(cfg);
    if (!tp_method_)
    {
        wbqp_init(&cfg, &qp_);
        qp_initialized_ = true;
    }
    else
    {
        initTaskPriorityController();
        qp_initialized_ = true;
    }
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

    auto cb_group_sub_pose_goal = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = cb_group_sub_pose_goal;
    sub_pose_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic_tcp_pose_goal_, 1, std::bind(&WbqpControllerNode::poseGoalCb, this, std::placeholders::_1), sub_options);

    auto cb_group_sub_arm_vel = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = cb_group_sub_arm_vel;
    sub_arm_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        topic_arm_vel_, 1, std::bind(&WbqpControllerNode::armVelCb, this, std::placeholders::_1), sub_options);

    auto cb_group_sub_obstacles = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = cb_group_sub_obstacles;
    sub_obstacles_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        topic_obstacle_points_, 1,
        std::bind(&WbqpControllerNode::obstaclePointsCb, this, std::placeholders::_1), sub_options);

    pub_cmd_vel_      = this->create_publisher<geometry_msgs::msg::Twist>(topic_cmd_vel_, 1);
    pub_q_speed_      = this->create_publisher<sensor_msgs::msg::JointState>(topic_q_speed_, 1);
    pub_arm_vel_cmd_  = this->create_publisher<geometry_msgs::msg::Twist>(topic_arm_vel_cmd_, 1);
    pub_emergency_state_ = this->create_publisher<std_msgs::msg::Bool>(topic_emergency_state_, 1);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    pub_base_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mobile_manipulator/base_pose", 1);

    qp_switch_srv_ = this->create_service<std_srvs::srv::SetBool>("~/enable_qp",
            std::bind(&WbqpControllerNode::onEnableQp, this, std::placeholders::_1, std::placeholders::_2));
    emergency_stop_srv_ = this->create_service<std_srvs::srv::SetBool>(
        emergency_stop_service_name_,
        std::bind(&WbqpControllerNode::onEmergencyStop, this, std::placeholders::_1, std::placeholders::_2));

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
  this->declare_parameter("topics.arm_vel_cmd", "/manipulator/cmd_vel");
  this->declare_parameter("topics.tcp_pose_goal", "/mobile_manipulator/tcp_pose_goal");
  this->declare_parameter("topics.obstacle_points", "/mobile_manipulator/obstacle_points");
  this->declare_parameter("topics.emergency_state", "/mobile_platform/emergency_stop_active");
  this->declare_parameter("services.emergency_stop", "/mobile_platform/emergency_stop");
  this->declare_parameter("control.dt",          0.02);

  topic_joint_state_ = this->get_parameter("topics.joint_state").as_string();
  topic_twist_cmd_   = this->get_parameter("topics.twist_cmd").as_string();
  topic_cmd_vel_     = this->get_parameter("topics.cmd_vel").as_string();
  topic_q_speed_     = this->get_parameter("topics.q_speed").as_string();
  topic_arm_vel_     = this->get_parameter("topics.arm_vel").as_string();
  topic_arm_vel_cmd_ = this->get_parameter("topics.arm_vel_cmd").as_string();
  topic_tcp_pose_goal_ = this->get_parameter("topics.tcp_pose_goal").as_string();
  topic_obstacle_points_ = this->get_parameter("topics.obstacle_points").as_string();
  topic_emergency_state_ = this->get_parameter("topics.emergency_state").as_string();
  emergency_stop_service_name_ = this->get_parameter("services.emergency_stop").as_string();
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
  this->declare_parameter<bool>("tp_method", false);
  tp_method_ = this->get_parameter("tp_method").as_bool();

  // --- C++ Jacobian parameters ---
  this->declare_parameter<bool>("jacobian.use_jac_ros2", false);
  this->declare_parameter<bool>("jacobian.jac_to_world", false);
  use_jac_ros2_ = this->get_parameter("jacobian.use_jac_ros2").as_bool();
  jac_to_world_ = this->get_parameter("jacobian.jac_to_world").as_bool();

  auto toLower = [](std::string value) {
    std::transform(
        value.begin(),
        value.end(),
        value.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
  };

  // --- Legacy singularity keys (backward compatibility) ---
  this->declare_parameter<bool>("singularity.enable", false);
  this->declare_parameter<double>("singularity.min_threshold", 1.0e-4);
  this->declare_parameter<std::string>("singularity.method", "yoshikawa");
  const bool legacy_singularity_enable = this->get_parameter("singularity.enable").as_bool();
  const double legacy_singularity_min_threshold = this->get_parameter("singularity.min_threshold").as_double();
  const std::string legacy_singularity_method = toLower(this->get_parameter("singularity.method").as_string());

  // --- QP singularity handling (qp.singularity.*) ---
  this->declare_parameter<bool>("qp.singularity.enable", legacy_singularity_enable);
  this->declare_parameter<double>("qp.singularity.min_threshold", legacy_singularity_min_threshold);
  this->declare_parameter<std::string>("qp.singularity.method", legacy_singularity_method);
  qp_singularity_enable_ = this->get_parameter("qp.singularity.enable").as_bool();
  qp_singularity_min_threshold_ = this->get_parameter("qp.singularity.min_threshold").as_double();
  if (qp_singularity_min_threshold_ < 0.0)
  {
    RCLCPP_WARN(this->get_logger(),
                "qp.singularity.min_threshold is negative (%.3e). Clamping to 0.0.",
                qp_singularity_min_threshold_);
    qp_singularity_min_threshold_ = 0.0;
  }
  qp_singularity_method_name_ = toLower(this->get_parameter("qp.singularity.method").as_string());
  if (qp_singularity_method_name_ == "det")
  {
    qp_singularity_method_ = SingularityMethod::DET;
  }
  else if (qp_singularity_method_name_ == "yoshikawa")
  {
    qp_singularity_method_ = SingularityMethod::YOSHIKAWA;
  }
  else
  {
    RCLCPP_WARN(this->get_logger(),
                "Unknown qp.singularity.method '%s'. Falling back to 'yoshikawa'.",
                qp_singularity_method_name_.c_str());
    qp_singularity_method_ = SingularityMethod::YOSHIKAWA;
    qp_singularity_method_name_ = "yoshikawa";
  }

  // --- TP-specific parameters (with backward-compatible fallbacks) ---
  this->declare_parameter<double>("tp.solver.lambda", 1.0e-4);
  this->declare_parameter<std::string>("tp.solver.pinv_method", "svd");
  tp_solver_lambda_ = this->get_parameter("tp.solver.lambda").as_double();
  tp_solver_pinv_method_ = this->get_parameter("tp.solver.pinv_method").as_string();
  std::transform(
      tp_solver_pinv_method_.begin(),
      tp_solver_pinv_method_.end(),
      tp_solver_pinv_method_.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (tp_solver_pinv_method_ != "svd" && tp_solver_pinv_method_ != "ldlt")
  {
    RCLCPP_WARN(this->get_logger(),
                "Unknown tp.solver.pinv_method '%s'. Falling back to 'svd'.",
                tp_solver_pinv_method_.c_str());
    tp_solver_pinv_method_ = "svd";
  }

  this->declare_parameter<int64_t>("tp.priorities.tracking", 1);
  this->declare_parameter<int64_t>("tp.priorities.joint_limits", 2);
  this->declare_parameter<int64_t>("tp.priorities.tcp_z", 3);
  this->declare_parameter<int64_t>("tp.priorities.singularity", 4);
  this->declare_parameter<int64_t>("tp.priorities.collision", 5);
  tp_priority_tracking_ = static_cast<int>(this->get_parameter("tp.priorities.tracking").as_int());
  tp_priority_joint_limits_ = static_cast<int>(this->get_parameter("tp.priorities.joint_limits").as_int());
  tp_priority_tcp_z_ = static_cast<int>(this->get_parameter("tp.priorities.tcp_z").as_int());
  tp_priority_singularity_ = static_cast<int>(this->get_parameter("tp.priorities.singularity").as_int());
  tp_priority_collision_ = static_cast<int>(this->get_parameter("tp.priorities.collision").as_int());

  this->declare_parameter("tp.tracking.kp", std::vector<double>{2,2,2,2,2,2});
  this->declare_parameter("tp.tracking.ki", std::vector<double>{0,0,0,0,0,0});
  this->declare_parameter("tp.tracking.v_limit", std::vector<double>{1,1,1,1,1,1});
  this->declare_parameter<std::string>("tp.mode", "speed");
  this->declare_parameter<std::string>("tp.pose_sub_mode", "increment");
  {
    const auto kp = this->get_parameter("tp.tracking.kp").as_double_array();
    const auto ki = this->get_parameter("tp.tracking.ki").as_double_array();
    const auto v_limit = this->get_parameter("tp.tracking.v_limit").as_double_array();
    for (int i = 0; i < 6; ++i)
    {
      tp_tracking_kp_[i] = (i < static_cast<int>(kp.size())) ? kp[i] : 2.0;
      tp_tracking_ki_[i] = (i < static_cast<int>(ki.size())) ? ki[i] : 0.0;
      tp_tracking_v_limit_[i] = (i < static_cast<int>(v_limit.size())) ? v_limit[i] : 1.0;
    }
  }
  {
    std::string tp_mode = toLower(this->get_parameter("tp.mode").as_string());
    std::string tp_pose_sub_mode = toLower(this->get_parameter("tp.pose_sub_mode").as_string());

    if (tp_mode == "pose")
    {
      tp_tracking_mode_ = TpTrackingMode::POSE;
    }
    else
    {
      if (tp_mode != "speed")
      {
        RCLCPP_WARN(this->get_logger(),
                    "Unknown tp.mode '%s'. Falling back to 'speed'.",
                    tp_mode.c_str());
      }
      tp_tracking_mode_ = TpTrackingMode::SPEED;
    }

    if (tp_pose_sub_mode == "input")
    {
      tp_pose_sub_mode_ = TpPoseSubMode::INPUT;
    }
    else
    {
      if (tp_pose_sub_mode != "increment")
      {
        RCLCPP_WARN(this->get_logger(),
                    "Unknown tp.pose_sub_mode '%s'. Falling back to 'increment'.",
                    tp_pose_sub_mode.c_str());
      }
      tp_pose_sub_mode_ = TpPoseSubMode::INCREMENT;
    }
  }

  this->declare_parameter<double>("tp.joint_limits.k", 2.0);
  this->declare_parameter<double>("tp.joint_limits.d_act", 0.2);
  this->declare_parameter<double>("tp.joint_limits.margin", 0.05);
  tp_joint_limits_k_ = this->get_parameter("tp.joint_limits.k").as_double();
  tp_joint_limits_d_act_ = this->get_parameter("tp.joint_limits.d_act").as_double();
  tp_joint_limits_margin_ = this->get_parameter("tp.joint_limits.margin").as_double();

  const double mu_min_fallback = std::max(1.0e-9, qp_singularity_min_threshold_);
  this->declare_parameter<bool>("tp.singularity.enable", legacy_singularity_enable);
  this->declare_parameter<std::string>("tp.singularity.method", qp_singularity_method_name_);
  this->declare_parameter<double>("tp.singularity.mu_min", mu_min_fallback);
  this->declare_parameter<double>("tp.singularity.mu_max", mu_min_fallback * 1.25);
  this->declare_parameter<double>("tp.singularity.mu_safe", mu_min_fallback * 1.5);
  this->declare_parameter<double>("tp.singularity.k", 1.0);
  this->declare_parameter<double>("tp.singularity.fd_eps", 1.0e-4);
  tp_singularity_enable_ = this->get_parameter("tp.singularity.enable").as_bool();
  tp_singularity_method_ = this->get_parameter("tp.singularity.method").as_string();
  tp_singularity_method_ = toLower(tp_singularity_method_);
  tp_singularity_mu_min_ = this->get_parameter("tp.singularity.mu_min").as_double();
  tp_singularity_mu_max_ = this->get_parameter("tp.singularity.mu_max").as_double();
  tp_singularity_mu_safe_ = this->get_parameter("tp.singularity.mu_safe").as_double();
  tp_singularity_k_ = this->get_parameter("tp.singularity.k").as_double();
  tp_singularity_fd_eps_ = this->get_parameter("tp.singularity.fd_eps").as_double();

  // --- Legacy shared collision keys (backward compatibility) ---
  this->declare_parameter<bool>("collision.enable", false);
  this->declare_parameter<double>("collision.d_safe", 0.15);
  this->declare_parameter<double>("collision.d_act", 0.30);
  this->declare_parameter<double>("collision.k", 3.0);
  this->declare_parameter<int64_t>("collision.max_constraints", 0);
  this->declare_parameter<double>("collision.base.size_x", 0.70);
  this->declare_parameter<double>("collision.base.size_y", 0.50);
  this->declare_parameter<double>("collision.base.delta_h", 0.05);
  this->declare_parameter<int64_t>("collision.samples_per_link", 0);
  this->declare_parameter<bool>("collision.include_tcp", true);
  this->declare_parameter<bool>("collision.include_joint_endpoints", true);
  const bool legacy_collision_enable = this->get_parameter("collision.enable").as_bool();
  const double legacy_collision_d_safe = this->get_parameter("collision.d_safe").as_double();
  const double legacy_collision_d_act = this->get_parameter("collision.d_act").as_double();
  const double legacy_collision_k = this->get_parameter("collision.k").as_double();
  const int legacy_collision_max_constraints = static_cast<int>(this->get_parameter("collision.max_constraints").as_int());
  const double legacy_collision_base_size_x = this->get_parameter("collision.base.size_x").as_double();
  const double legacy_collision_base_size_y = this->get_parameter("collision.base.size_y").as_double();
  const double legacy_collision_base_delta_h = this->get_parameter("collision.base.delta_h").as_double();
  const int legacy_collision_samples_per_link = static_cast<int>(this->get_parameter("collision.samples_per_link").as_int());
  const bool legacy_collision_include_tcp = this->get_parameter("collision.include_tcp").as_bool();
  const bool legacy_collision_include_joint_endpoints = this->get_parameter("collision.include_joint_endpoints").as_bool();

  // --- QP collision (qp.collision.*): hard point-to-point inequalities ---
  this->declare_parameter<bool>("qp.collision.enable", legacy_collision_enable);
  this->declare_parameter<double>("qp.collision.d_safe", legacy_collision_d_safe);
  this->declare_parameter<int64_t>("qp.collision.max_constraints", legacy_collision_max_constraints);
  this->declare_parameter<double>("qp.collision.base.size_x", legacy_collision_base_size_x);
  this->declare_parameter<double>("qp.collision.base.size_y", legacy_collision_base_size_y);
  this->declare_parameter<double>("qp.collision.base.delta_h", legacy_collision_base_delta_h);
  this->declare_parameter<int64_t>("qp.collision.samples_per_link", legacy_collision_samples_per_link);
  this->declare_parameter<bool>("qp.collision.include_tcp", legacy_collision_include_tcp);
  this->declare_parameter<bool>("qp.collision.include_joint_endpoints", legacy_collision_include_joint_endpoints);
  this->declare_parameter<bool>("qp.collision.use_closest_obstacle", true);
  qp_collision_enable_ = this->get_parameter("qp.collision.enable").as_bool();
  qp_collision_d_safe_ = this->get_parameter("qp.collision.d_safe").as_double();
  qp_collision_max_constraints_ = static_cast<int>(this->get_parameter("qp.collision.max_constraints").as_int());
  qp_collision_base_size_x_ = this->get_parameter("qp.collision.base.size_x").as_double();
  qp_collision_base_size_y_ = this->get_parameter("qp.collision.base.size_y").as_double();
  qp_collision_base_delta_h_ = this->get_parameter("qp.collision.base.delta_h").as_double();
  qp_collision_samples_per_link_ = static_cast<int>(this->get_parameter("qp.collision.samples_per_link").as_int());
  qp_collision_include_tcp_ = this->get_parameter("qp.collision.include_tcp").as_bool();
  qp_collision_include_joint_endpoints_ = this->get_parameter("qp.collision.include_joint_endpoints").as_bool();
  qp_collision_use_closest_obstacle_ = this->get_parameter("qp.collision.use_closest_obstacle").as_bool();
  qp_collision_max_constraints_ = std::max(0, qp_collision_max_constraints_);
  qp_collision_samples_per_link_ = std::max(0, qp_collision_samples_per_link_);
  qp_collision_base_size_x_ = std::max(0.0, qp_collision_base_size_x_);
  qp_collision_base_size_y_ = std::max(0.0, qp_collision_base_size_y_);
  qp_collision_base_delta_h_ = std::max(0.0, qp_collision_base_delta_h_);

  // --- TP collision (tp.collision.*): activation/recovery parameters + point model ---
  this->declare_parameter<bool>("tp.collision.enable", legacy_collision_enable);
  this->declare_parameter<double>("tp.collision.d_safe", legacy_collision_d_safe);
  this->declare_parameter<double>("tp.collision.d_act", std::max(legacy_collision_d_safe, legacy_collision_d_act));
  this->declare_parameter<double>("tp.collision.k", legacy_collision_k);
  this->declare_parameter<int64_t>("tp.collision.max_constraints", legacy_collision_max_constraints);
  this->declare_parameter<double>("tp.collision.base.size_x", legacy_collision_base_size_x);
  this->declare_parameter<double>("tp.collision.base.size_y", legacy_collision_base_size_y);
  this->declare_parameter<double>("tp.collision.base.delta_h", legacy_collision_base_delta_h);
  this->declare_parameter<int64_t>("tp.collision.samples_per_link", legacy_collision_samples_per_link);
  this->declare_parameter<bool>("tp.collision.include_tcp", legacy_collision_include_tcp);
  this->declare_parameter<bool>("tp.collision.include_joint_endpoints", legacy_collision_include_joint_endpoints);
  this->declare_parameter<bool>("tp.collision.use_closest_obstacle", true);
  this->declare_parameter("tp.collision.robot_points", std::vector<int64_t>{0, 1}); // legacy, ignored
  tp_collision_enable_ = this->get_parameter("tp.collision.enable").as_bool();
  tp_collision_d_safe_ = this->get_parameter("tp.collision.d_safe").as_double();
  tp_collision_d_act_ = this->get_parameter("tp.collision.d_act").as_double();
  tp_collision_k_ = this->get_parameter("tp.collision.k").as_double();
  tp_collision_max_constraints_ = static_cast<int>(this->get_parameter("tp.collision.max_constraints").as_int());
  tp_collision_base_size_x_ = this->get_parameter("tp.collision.base.size_x").as_double();
  tp_collision_base_size_y_ = this->get_parameter("tp.collision.base.size_y").as_double();
  tp_collision_base_delta_h_ = this->get_parameter("tp.collision.base.delta_h").as_double();
  tp_collision_samples_per_link_ = static_cast<int>(this->get_parameter("tp.collision.samples_per_link").as_int());
  tp_collision_include_tcp_ = this->get_parameter("tp.collision.include_tcp").as_bool();
  tp_collision_include_joint_endpoints_ = this->get_parameter("tp.collision.include_joint_endpoints").as_bool();
  tp_collision_use_closest_obstacle_ = this->get_parameter("tp.collision.use_closest_obstacle").as_bool();
  tp_collision_max_constraints_ = std::max(0, tp_collision_max_constraints_);
  tp_collision_samples_per_link_ = std::max(0, tp_collision_samples_per_link_);
  tp_collision_d_act_ = std::max(tp_collision_d_safe_, tp_collision_d_act_);
  tp_collision_base_size_x_ = std::max(0.0, tp_collision_base_size_x_);
  tp_collision_base_size_y_ = std::max(0.0, tp_collision_base_size_y_);
  tp_collision_base_delta_h_ = std::max(0.0, tp_collision_base_delta_h_);

  // --- Legacy shared TCP-z guard keys (backward compatibility) ---
  this->declare_parameter<bool>("tcp_z_guard.enable", false);
  this->declare_parameter<double>("tcp_z_guard.z_min", 0.03);
  this->declare_parameter<double>("tcp_z_guard.z_act", 0.06);
  this->declare_parameter<double>("tcp_z_guard.k", 2.0);
  const bool legacy_tcp_z_guard_enable = this->get_parameter("tcp_z_guard.enable").as_bool();
  const double legacy_tcp_z_guard_z_min = this->get_parameter("tcp_z_guard.z_min").as_double();
  const double legacy_tcp_z_guard_z_act = this->get_parameter("tcp_z_guard.z_act").as_double();
  const double legacy_tcp_z_guard_k = this->get_parameter("tcp_z_guard.k").as_double();

  // --- QP TCP-z guard (qp.tcp_z_guard.*): hard lower-bound inequality only ---
  this->declare_parameter<bool>("qp.tcp_z_guard.enable", legacy_tcp_z_guard_enable);
  this->declare_parameter<double>("qp.tcp_z_guard.z_min", legacy_tcp_z_guard_z_min);
  qp_tcp_z_guard_enable_ = this->get_parameter("qp.tcp_z_guard.enable").as_bool();
  qp_tcp_z_guard_z_min_ = this->get_parameter("qp.tcp_z_guard.z_min").as_double();

  // --- TP TCP-z guard (tp.tcp_z_guard.*): activation/recovery task ---
  this->declare_parameter<bool>("tp.tcp_z_guard.enable", legacy_tcp_z_guard_enable);
  this->declare_parameter<double>("tp.tcp_z_guard.z_min", legacy_tcp_z_guard_z_min);
  this->declare_parameter<double>("tp.tcp_z_guard.z_act", std::max(legacy_tcp_z_guard_z_min, legacy_tcp_z_guard_z_act));
  this->declare_parameter<double>("tp.tcp_z_guard.k", legacy_tcp_z_guard_k);
  tp_tcp_z_guard_enable_ = this->get_parameter("tp.tcp_z_guard.enable").as_bool();
  tp_tcp_z_guard_z_min_ = this->get_parameter("tp.tcp_z_guard.z_min").as_double();
  tp_tcp_z_guard_z_act_ = this->get_parameter("tp.tcp_z_guard.z_act").as_double();
  tp_tcp_z_guard_k_ = this->get_parameter("tp.tcp_z_guard.k").as_double();
  tp_tcp_z_guard_z_act_ = std::max(tp_tcp_z_guard_z_min_, tp_tcp_z_guard_z_act_);

  // --- DH parameters (6 joints: a, d, alpha) ---
  const std::array<std::string, 6> joint_names = {"joint1","joint2","joint3","joint4","joint5","joint6"};
  // UR5e defaults
  const std::array<double, 6> default_a     = { 0.0,    -0.425, -0.3922,  0.0,    0.0,    0.0   };
  const std::array<double, 6> default_d     = { 0.1625,  0.0,    0.0,     0.1333, 0.0997, 0.0996};
  const std::array<double, 6> default_alpha = { M_PI/2,  0.0,    0.0,     M_PI/2,-M_PI/2, 0.0   };
  for (int i = 0; i < 6; ++i) {
    this->declare_parameter("kinematics.dh." + joint_names[i] + ".a",     default_a[i]);
    this->declare_parameter("kinematics.dh." + joint_names[i] + ".d",     default_d[i]);
    this->declare_parameter("kinematics.dh." + joint_names[i] + ".alpha", default_alpha[i]);
    dh_params_[i].a     = this->get_parameter("kinematics.dh." + joint_names[i] + ".a").as_double();
    dh_params_[i].d     = this->get_parameter("kinematics.dh." + joint_names[i] + ".d").as_double();
    dh_params_[i].alpha = this->get_parameter("kinematics.dh." + joint_names[i] + ".alpha").as_double();
  }

  // --- TCP offset [tx, ty, tz, rx, ry, rz] ---
  this->declare_parameter("kinematics.tcp_offset", std::vector<double>{0,0,0,0,0,0});
  auto tcp = this->get_parameter("kinematics.tcp_offset").as_double_array();
  for (int i = 0; i < 6 && i < (int)tcp.size(); ++i) { tcp_offset_[i] = tcp[i]; }

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
    RCLCPP_INFO(this->get_logger(), "  tcp_pose_goal: %s", topic_tcp_pose_goal_.c_str());
    RCLCPP_INFO(this->get_logger(), "  emergency srv: %s", emergency_stop_service_name_.c_str());
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

    // --- QP weights & limits ---
    RCLCPP_INFO(this->get_logger(), "QP Parameters:");
    RCLCPP_INFO(this->get_logger(), "  tp_method  = %s", tp_method_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  use_native = %s", use_native_qp_ ? "true" : "false");
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

    // --- Jacobian mode ---
    RCLCPP_INFO(this->get_logger(), "Jacobian:");
    RCLCPP_INFO(this->get_logger(), "  use_jac_ros2 = %s", use_jac_ros2_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  jac_to_world = %s", jac_to_world_ ? "true" : "false");
    if (use_jac_ros2_) {
        RCLCPP_INFO(this->get_logger(), "  DH parameters:");
        for (int i = 0; i < 6; ++i) {
            RCLCPP_INFO(this->get_logger(), "    joint%d: a=%.4f  d=%.4f  alpha=%.4f",
                        i+1, dh_params_[i].a, dh_params_[i].d, dh_params_[i].alpha);
        }
        RCLCPP_INFO(this->get_logger(), "  TCP offset: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                    tcp_offset_[0], tcp_offset_[1], tcp_offset_[2],
                    tcp_offset_[3], tcp_offset_[4], tcp_offset_[5]);
    }
    RCLCPP_INFO(this->get_logger(), "QP Singularity:");
    RCLCPP_INFO(this->get_logger(), "  enable        = %s", qp_singularity_enable_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  min_threshold = %.6e", qp_singularity_min_threshold_);
    RCLCPP_INFO(this->get_logger(), "  method        = %s", qp_singularity_method_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "QP Collision:");
    RCLCPP_INFO(this->get_logger(), "  enable        = %s", qp_collision_enable_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  d_safe        = %.4f", qp_collision_d_safe_);
    RCLCPP_INFO(this->get_logger(), "  closest_only  = %s", qp_collision_use_closest_obstacle_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  base          = [size_x=%.3f size_y=%.3f delta_h=%.3f], samples_per_link=%d",
                qp_collision_base_size_x_, qp_collision_base_size_y_,
                qp_collision_base_delta_h_, qp_collision_samples_per_link_);
    RCLCPP_INFO(this->get_logger(), "QP TCP z guard:");
    RCLCPP_INFO(this->get_logger(), "  enable / z_min = [%s, %.4f]",
                qp_tcp_z_guard_enable_ ? "true" : "false", qp_tcp_z_guard_z_min_);
    RCLCPP_INFO(this->get_logger(), "TP Parameters:");
    RCLCPP_INFO(this->get_logger(), "  solver.lambda      = %.6e", tp_solver_lambda_);
    RCLCPP_INFO(this->get_logger(), "  solver.pinv_method = %s", tp_solver_pinv_method_.c_str());
    RCLCPP_INFO(this->get_logger(), "  priorities         = [tracking:%d, joint_limits:%d, tcp_z:%d, singularity:%d, collision:%d]",
                tp_priority_tracking_, tp_priority_joint_limits_, tp_priority_tcp_z_,
                tp_priority_singularity_, tp_priority_collision_);
    RCLCPP_INFO(this->get_logger(), "  tracking.v_limit   = [%.3f %.3f %.3f %.3f %.3f %.3f]",
                tp_tracking_v_limit_[0], tp_tracking_v_limit_[1], tp_tracking_v_limit_[2],
                tp_tracking_v_limit_[3], tp_tracking_v_limit_[4], tp_tracking_v_limit_[5]);
    RCLCPP_INFO(this->get_logger(), "  mode               = %s",
                (tp_tracking_mode_ == TpTrackingMode::POSE) ? "pose" : "speed");
    RCLCPP_INFO(this->get_logger(), "  pose_sub_mode      = %s",
                (tp_pose_sub_mode_ == TpPoseSubMode::INPUT) ? "input" : "increment");
    RCLCPP_INFO(this->get_logger(), "  singularity.enable = %s", tp_singularity_enable_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  singularity.method = %s", tp_singularity_method_.c_str());
    RCLCPP_INFO(this->get_logger(), "  collision.enable   = %s", tp_collision_enable_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  collision.closest  = %s", tp_collision_use_closest_obstacle_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  collision.base     = [size_x=%.3f size_y=%.3f delta_h=%.3f], samples_per_link=%d",
                tp_collision_base_size_x_, tp_collision_base_size_y_, tp_collision_base_delta_h_,
                tp_collision_samples_per_link_);
    RCLCPP_INFO(this->get_logger(), "TCP z guard:");
    RCLCPP_INFO(this->get_logger(), "  enable             = %s", tp_tcp_z_guard_enable_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  z_min / z_act / k  = [%.4f, %.4f, %.3f]",
                tp_tcp_z_guard_z_min_, tp_tcp_z_guard_z_act_, tp_tcp_z_guard_k_);

    RCLCPP_INFO(this->get_logger(), "--------------------------");
}

// ------------------- MAIN LOOP ------------------- //
bool WbqpControllerNode::loopStep()
{
    // Always publish static TF
    static_tf_broadcaster_->sendTransform(tf_N2B_);

    if (mode_ == Mode::RUN)
    {
        const bool controller_ready = tp_method_ ? static_cast<bool>(tp_controller_) : qp_initialized_;
        bool command_ready = have_twist_;
        if (tp_method_ &&
            tp_tracking_mode_ == TpTrackingMode::POSE &&
            tp_pose_sub_mode_ == TpPoseSubMode::INPUT)
        {
            command_ready = have_pose_goal_input_;
        }

        if (!controller_ready || !have_js_ || !command_ready || !qp_enabled_)
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

    // --- Jacobian computation ---
    struct1_T in{};
    if (use_jac_ros2_)
    {
        double J6x9_colmajor[54];
        computeJacobianRos2(J6x9_colmajor);
        std::memset(&in, 0, sizeof(in));
        for (int i = 0; i < 54; ++i) { in.J[i] = J6x9_colmajor[i]; }
        for (int i = 0; i < 6; ++i) { in.q[i] = q_pos_[i]; }
        for (int i = 0; i < 6; ++i) { in.u_star[i] = u_star_[i]; }
        for (int i = 0; i < 9; ++i) { in.dotq_prev[i] = dotq_prev_[i]; }
        in.dt = dt_;
    }
    else
    {
        double in1[15];
        buildIn15(in1);
        double A6x12_colmajor[72];
        WholeBodyJacobian(in1, A6x12_colmajor);
        fillInputStruct(in, A6x12_colmajor);
    }

    double x_opt[9];
    bool solver_ok = true;
    bool zero_output_this_cycle = false;
    try
    {
        if (tp_method_)
        {
            solve_tp(x_opt);
        }
        else if (use_native_qp_)
        {
            solve_qp_native(in, in.J, x_opt);
        }
        else
        {
            struct2_T dbg_out{};
            wbqp_solve(reinterpret_cast<const struct0_T *>(&qp_), &in, x_opt, &dbg_out);
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Controller solver failed: %s", e.what());
        solver_ok = false;
        if (std::strstr(e.what(), "inconsistent variable bounds (lb > ub)") != nullptr)
        {
            zero_output_this_cycle = true;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "QP infeasible due to bounds mismatch. Publishing zero commands for this cycle.");
        }
    }

    if (!solver_ok)
    {
        if (zero_output_this_cycle)
        {
            for (double &v : x_opt) { v = 0.0; }
        }
        else
        {
            for (int i=0;i<9;++i) { x_opt[i] = dotq_prev_[i]; }
        }
    }

    bool xopt_ok = checkXoptValues(x_opt);
    if (!xopt_ok)
    {
        RCLCPP_ERROR(this->get_logger(), "QP solver returned invalid values (inf or nan). Skipping this cycle.");
        for (int i=0;i<9;++i) { x_opt[i] = dotq_prev_[i]; }
    }

    if (qp_singularity_enable_ && !tp_method_)
    {
        enforceSingularityConstraint(x_opt);
    }

    if (emergency_stop_active_.load()) {
        for (double &v : x_opt) {
            v = 0.0;
        }
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

void WbqpControllerNode::poseGoalCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    const auto goal = poseMsgToTpPose(msg->pose);
    {
        std::lock_guard<std::mutex> lock(tp_pose_goal_mtx_);
        tp_pose_goal_input_ = goal;
    }
    have_pose_goal_input_ = true;
}

void WbqpControllerNode::armVelCb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    arm_vel_[0]=msg->linear.x;  arm_vel_[1]=msg->linear.y;  arm_vel_[2]=msg->linear.z;
    arm_vel_[3]=msg->angular.x; arm_vel_[4]=msg->angular.y; arm_vel_[5]=msg->angular.z;
}

void WbqpControllerNode::obstaclePointsCb(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    std::vector<Eigen::Vector3d> points;
    points.reserve(msg->poses.size());
    for (const auto& p : msg->poses)
    {
        points.emplace_back(p.position.x, p.position.y, p.position.z);
    }

    {
        std::lock_guard<std::mutex> lock(obstacle_points_mtx_);
        obstacle_points_world_ = std::move(points);
        obstacle_points_frame_ = msg->header.frame_id;
    }
}

void WbqpControllerNode::onEnableQp(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                         std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    qp_enabled_ = request->data;   // true = enable, false = disable
    response->success = true;
    response->message = qp_enabled_ ? "QP enabled" : "QP disabled";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void WbqpControllerNode::onEmergencyStop(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    emergency_stop_active_.store(request->data);
    std_msgs::msg::Bool state_msg;
    state_msg.data = request->data;
    pub_emergency_state_->publish(state_msg);
    response->success = true;
    response->message = request->data ? "Mobile platform emergency stop enabled"
                                      : "Mobile platform emergency stop released";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
}

// ------------------- MATLAB OPTIMIZER CONFIGURATION ------------------- //
void WbqpControllerNode::fillCfgStruct(struct10_T &cfg)
{
    std::memset(&cfg, 0, sizeof(cfg));

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
    // Legacy location (limits.*) kept for backward compatibility.
    this->declare_parameter("limits.qddot_max", 2.0);
    this->declare_parameter("limits.a_lin_max", 1.0);
    this->declare_parameter("limits.alpha_max", 1.0);
    const double legacy_qddot_max = this->get_parameter("limits.qddot_max").as_double();
    const double legacy_a_lin_max = this->get_parameter("limits.a_lin_max").as_double();
    const double legacy_alpha_max = this->get_parameter("limits.alpha_max").as_double();

    this->declare_parameter("qp.qddot_max",  legacy_qddot_max);
    this->declare_parameter("qp.a_lin_max",  legacy_a_lin_max);
    this->declare_parameter("qp.alpha_max",  legacy_alpha_max);

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

    // Cache YAML values for native runtime solver.
    qp_beta_arm_ = cfg.beta_arm;
    qp_alpha_xy_ = cfg.alpha_xy;
    qp_alpha_yaw_ = cfg.alpha_yaw;
    qp_w_lin_ = cfg.w_lin;
    qp_w_ang_ = cfg.w_ang;
    qp_nu_ = cfg.nu;
    qp_max_dotq_ = cfg.max_dotq;
    qp_max_V_ = cfg.max_V;
    qp_max_Omegaz_ = cfg.max_Omegaz;
    qp_qddot_max_ = cfg.qddot_max;
    qp_a_lin_max_ = cfg.a_lin_max;
    qp_alpha_max_ = cfg.alpha_max;

    // --- Joint limits arrays ---
    this->declare_parameter("limits.qmin", std::vector<double>{-10.0, -120.0, -135.0, -135.0, 0.0, -180.0});
    this->declare_parameter("limits.qmax", std::vector<double>{ 90.0,  -85.0,    0.0,   90.0, 110.0, 180.0});
    auto qmin = this->get_parameter("limits.qmin").as_double_array();
    auto qmax = this->get_parameter("limits.qmax").as_double_array();
    constexpr double kDegToRad = M_PI / 180.0;
    for (int i=0;i<6;++i) {
        cfg.qmin[i] = qmin[i] * kDegToRad;
        cfg.qmax[i] = qmax[i] * kDegToRad;
    }

    // Native qp solver: store qmin/qmax for warm start
    for (int i=0;i<6;++i) {
        in_qmin_from_cfg_or_params_[i] = cfg.qmin[i];
        in_qmax_from_cfg_or_params_[i] = cfg.qmax[i];
    }
}

void WbqpControllerNode::fillInputStruct(struct1_T &in, const double J6x12_colmajor[72]) const
{
    std::memset(&in, 0, sizeof(in));

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

// ------------------- C++ JACOBIAN (wb_jac) ------------------- //
void WbqpControllerNode::initKinematics()
{
    // In pure vendor-QP mode, wb_jac is not used in the control loop.
    if (!use_jac_ros2_ && !tp_method_)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Using vendor WholeBodyJacobian path (use_jac_ros2=false, tp_method=false).");
        return;
    }

    // Set DH params from config
    kin_.setDH(dh_params_);

    // Set fixed transform N->B from P_N2B and theta_N2B (RPY)
    const double cr = std::cos(theta_N2B_[0]), sr = std::sin(theta_N2B_[0]);
    const double cp = std::cos(theta_N2B_[1]), sp = std::sin(theta_N2B_[1]);
    const double cy = std::cos(theta_N2B_[2]), sy = std::sin(theta_N2B_[2]);

    Eigen::Matrix3d R_NB;
    R_NB << cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr,
            sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr,
              -sp,        cp*sr,              cp*cr;

    Eigen::Vector3d p_NB(P_N2B_[0], P_N2B_[1], P_N2B_[2]);
    kin_.setFixedTransformNB(R_NB, p_NB);

    // Set TCP offset
    kin_.setTcpOffset(tcp_offset_[0], tcp_offset_[1], tcp_offset_[2],
                      tcp_offset_[3], tcp_offset_[4], tcp_offset_[5]);

    RCLCPP_INFO(
        this->get_logger(),
        "C++ Jacobian (wb_jac) initialized [%s frame].",
        jac_to_world_ ? "world" : "body");
}

void WbqpControllerNode::computeJacobianRos2(double J6x9_colmajor[54]) const
{
    Eigen::Matrix<double, 6, 1> q;
    for (int i = 0; i < 6; ++i) q(i) = q_pos_[i];

    Eigen::Matrix<double, 6, 9> J;

    if (jac_to_world_)
    {
        MobileManipulatorKinematics::BasePose2D basePose;
        basePose.px  = x_base_;
        basePose.py  = y_base_;
        basePose.yaw = theta_W2N_[2];
        J = kin_.jacobianWorld(q, basePose);
    }
    else
    {
        J = kin_.jacobianBody(q);
    }

    // Store in column-major order (Eigen default is ColMajor)
    Eigen::Map<Eigen::Matrix<double, 6, 9, Eigen::ColMajor>> J_out(J6x9_colmajor);
    J_out = J;
}

double WbqpControllerNode::computeSingularityMetricFromArmJacobian(
    const Eigen::Matrix<double,6,6>& J_arm) const
{
    if (qp_singularity_method_ == SingularityMethod::DET)
    {
        return std::abs(J_arm.determinant());
    }

    const Eigen::Matrix<double,6,6> JJt = J_arm * J_arm.transpose();
    const double det_jjt = JJt.determinant();
    return std::sqrt(std::max(0.0, det_jjt));
}

double WbqpControllerNode::computeSingularityMetricForQ(const std::array<double,6>& q_rad) const
{
    Eigen::Matrix<double,6,1> q;
    for (int i = 0; i < 6; ++i) { q(i) = q_rad[i]; }

    Eigen::Matrix<double,6,9> J;
    if (jac_to_world_)
    {
        MobileManipulatorKinematics::BasePose2D basePose;
        basePose.px = x_base_;
        basePose.py = y_base_;
        basePose.yaw = theta_W2N_[2];
        J = kin_.jacobianWorld(q, basePose);
    }
    else
    {
        J = kin_.jacobianBody(q);
    }

    const Eigen::Matrix<double,6,6> J_arm = J.block<6,6>(0,0);
    return computeSingularityMetricFromArmJacobian(J_arm);
}

void WbqpControllerNode::enforceSingularityConstraint(double x_opt[9])
{
    const double threshold = qp_singularity_min_threshold_;

    std::array<double,6> q_current{};
    for (int i = 0; i < 6; ++i) { q_current[i] = q_pos_[i]; }

    const double sigma_current = computeSingularityMetricForQ(q_current);
    if (sigma_current + 1.0e-12 < threshold)
    {
        for (int i = 0; i < 6; ++i) { x_opt[i] = 0.0; }
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Singularity metric %.6e is below threshold %.6e. Arm motion is frozen.",
            sigma_current,
            threshold);
        return;
    }

    auto sigma_at_scale = [&](double scale) {
        std::array<double,6> q_trial{};
        for (int i = 0; i < 6; ++i)
        {
            q_trial[i] = q_current[i] + (scale * x_opt[i] * dt_);
        }
        return computeSingularityMetricForQ(q_trial);
    };

    const double sigma_full_step = sigma_at_scale(1.0);
    if (sigma_full_step + 1.0e-12 >= threshold)
    {
        return;
    }

    double lo = 0.0;
    double hi = 1.0;
    for (int iter = 0; iter < 24; ++iter)
    {
        const double mid = 0.5 * (lo + hi);
        const double sigma_mid = sigma_at_scale(mid);
        if (sigma_mid + 1.0e-12 >= threshold)
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
    }

    for (int i = 0; i < 6; ++i) { x_opt[i] *= lo; }

    const double sigma_limited = sigma_at_scale(lo);
    if (sigma_limited + 1.0e-12 < threshold)
    {
        for (int i = 0; i < 6; ++i) { x_opt[i] = 0.0; }
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Unable to satisfy singularity threshold %.6e with commanded step. Arm motion is frozen.",
            threshold);
        return;
    }

    if (lo < 0.999)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Arm velocity scaled by %.3f to keep singularity metric above %.6e.",
            lo,
            threshold);
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
    nin.w_lin = qp_w_lin_;
    nin.w_ang = qp_w_ang_;
    nin.beta_arm = qp_beta_arm_;
    nin.alpha_xy = qp_alpha_xy_;
    nin.alpha_yaw = qp_alpha_yaw_;
    nin.nu = qp_nu_;
    nin.max_dotq = qp_max_dotq_;
    nin.max_V = qp_max_V_;
    nin.max_Omegaz = qp_max_Omegaz_;
    nin.qddot_max = qp_qddot_max_;
    nin.a_lin_max = qp_a_lin_max_;
    nin.alpha_max = qp_alpha_max_;

    CollisionPointModelConfig collision_cfg;
    collision_cfg.dh_params = dh_params_;
    collision_cfg.tcp_offset = tcp_offset_;
    collision_cfg.p_NB = Eigen::Vector3d(P_N2B_[0], P_N2B_[1], P_N2B_[2]);
    collision_cfg.R_NB = rpyToR(theta_N2B_[0], theta_N2B_[1], theta_N2B_[2]);
    collision_cfg.base_size_x = qp_collision_base_size_x_;
    collision_cfg.base_size_y = qp_collision_base_size_y_;
    collision_cfg.base_delta_h = qp_collision_base_delta_h_;
    collision_cfg.samples_per_link = qp_collision_samples_per_link_;
    collision_cfg.include_joint_endpoints = qp_collision_include_joint_endpoints_;
    collision_cfg.include_tcp = qp_collision_include_tcp_;

    std::array<double, 6> q_arm{};
    for (int i = 0; i < 6; ++i) { q_arm[i] = q_pos_[i]; }
    const RobotPointModelCache model = computeRobotPointModel(
        collision_cfg, x_base_, y_base_, theta_W2N_[2], q_arm);

    // TCP z hard guard in arm-base frame B: z_B(k+1) >= z_min.
    if (qp_tcp_z_guard_enable_)
    {
        std::array<double, 9> row{};
        for (int c = 0; c < 9; ++c) { row[c] = model.tcp_height_row_legacy(0, c); }
        const double rhs = (qp_tcp_z_guard_z_min_ - model.tcp_height_B) / std::max(1.0e-9, dt_);
        nin.extra_A_rows.push_back(row);
        nin.extra_l.push_back(rhs);
        nin.extra_u.push_back(1.0e20);
    }

    // Collision hard layer: linearized point-to-point constraints.
    // For each pair (robot_point i, obstacle point j):
    // d_ij(k+1) ≈ d_ij(k) + dt * n_ij^T J_i z  >= d_safe.
    // => n_ij^T J_i z >= (d_safe - d_ij)/dt.
    if (qp_collision_enable_)
    {
        std::vector<Eigen::Vector3d> obstacles;
        {
            std::lock_guard<std::mutex> lock(obstacle_points_mtx_);
            obstacles = obstacle_points_world_;
        }

        struct LinearizedCollisionRow
        {
            double distance = 0.0;
            std::array<double, 9> A{};
            double lower = 0.0;
        };
        std::vector<LinearizedCollisionRow> rows;
        rows.reserve(model.collision_point_ids.size() * std::max<std::size_t>(1, obstacles.size()));

        for (const int id : model.collision_point_ids)
        {
            const auto it = model.points.find(id);
            if (it == model.points.end())
            {
                continue;
            }
            const auto& rp = it->second;

            if (qp_collision_use_closest_obstacle_)
            {
                double best_d = std::numeric_limits<double>::infinity();
                Eigen::Vector3d best_obs = Eigen::Vector3d::Zero();
                bool found = false;
                for (const auto& obs : obstacles)
                {
                    const double d = (rp.p_world - obs).norm();
                    if (d < best_d)
                    {
                        best_d = d;
                        best_obs = obs;
                        found = true;
                    }
                }
                if (!found || best_d <= 1.0e-9)
                {
                    continue;
                }

                const Eigen::Vector3d diff = rp.p_world - best_obs;
                const Eigen::Vector3d n = diff / best_d;
                const Eigen::Matrix<double, 1, 9> Jn = n.transpose() * rp.J_world_legacy;

                LinearizedCollisionRow row;
                row.distance = best_d;
                for (int c = 0; c < 9; ++c) { row.A[c] = Jn(0, c); }
                row.lower = (qp_collision_d_safe_ - best_d) / std::max(1.0e-9, dt_);
                rows.push_back(row);
                continue;
            }

            for (const auto& obs : obstacles)
            {
                const Eigen::Vector3d diff = rp.p_world - obs;
                const double d = diff.norm();
                if (d <= 1.0e-9)
                {
                    continue;
                }

                const Eigen::Vector3d n = diff / d;
                const Eigen::Matrix<double, 1, 9> Jn = n.transpose() * rp.J_world_legacy;

                LinearizedCollisionRow row;
                row.distance = d;
                for (int c = 0; c < 9; ++c) { row.A[c] = Jn(0, c); }
                row.lower = (qp_collision_d_safe_ - d) / std::max(1.0e-9, dt_);
                rows.push_back(row);
            }
        }

        // Keep the most critical pairs first if capped.
        std::sort(rows.begin(), rows.end(),
                  [](const LinearizedCollisionRow& a, const LinearizedCollisionRow& b)
                  { return a.distance < b.distance; });
        const int keep =
            (qp_collision_max_constraints_ <= 0)
                ? static_cast<int>(rows.size())
                : std::min(static_cast<int>(rows.size()), qp_collision_max_constraints_);
        for (int i = 0; i < keep; ++i)
        {
            nin.extra_A_rows.push_back(rows[i].A);
            nin.extra_l.push_back(rows[i].lower);
            nin.extra_u.push_back(1.0e20);
        }
    }

    wbqp::NativeQpOutput zout = wbqp::NativeQpSolver::solve(nin);
    for (int i=0;i<9;++i) x_opt[i] = zout[i];
}

void WbqpControllerNode::initTaskPriorityController()
{
    CollisionPointModelConfig collision_cfg;
    collision_cfg.dh_params = dh_params_;
    collision_cfg.tcp_offset = tcp_offset_;
    collision_cfg.p_NB = Eigen::Vector3d(P_N2B_[0], P_N2B_[1], P_N2B_[2]);
    collision_cfg.R_NB = rpyToR(theta_N2B_[0], theta_N2B_[1], theta_N2B_[2]);
    collision_cfg.base_size_x = tp_collision_base_size_x_;
    collision_cfg.base_size_y = tp_collision_base_size_y_;
    collision_cfg.base_delta_h = tp_collision_base_delta_h_;
    collision_cfg.samples_per_link = tp_collision_samples_per_link_;
    collision_cfg.include_joint_endpoints = tp_collision_include_joint_endpoints_;
    collision_cfg.include_tcp = tp_collision_include_tcp_;

    // TP Jacobian switch:
    // use_jac_ros2=false -> vendor WholeBodyJacobian
    // use_jac_ros2=true  -> wb_jac ROS2 path (jac_to_world selects world/base)
    const bool tp_use_vendor_jacobian = !use_jac_ros2_;
    if (tp_use_vendor_jacobian && jac_to_world_)
    {
        RCLCPP_WARN(this->get_logger(),
                    "TP vendor Jacobian path selected (use_jac_ros2=false): jacobian.jac_to_world is ignored.");
    }
    auto kin_provider = std::make_unique<WbqpTpKinematicsProvider>(
        &kin_,
        jac_to_world_,
        tp_use_vendor_jacobian,
        std::array<double, 3>{P_N2B_[0], P_N2B_[1], P_N2B_[2]},
        std::array<double, 3>{theta_N2B_[0], theta_N2B_[1], theta_N2B_[2]},
        cols1based_,
        collision_cfg);

    std::vector<std::unique_ptr<tp_control::ITask>> tasks;

    tp_control::TrackingTask::Params tracking_params;
    tracking_params.priority = tp_priority_tracking_;
    for (int i = 0; i < 6; ++i)
    {
        tracking_params.Kp(i) = tp_tracking_kp_[i];
        tracking_params.Ki(i) = tp_tracking_ki_[i];
        tracking_params.v_limit(i) = tp_tracking_v_limit_[i];
    }
    tasks.push_back(std::make_unique<tp_control::TrackingTask>(tracking_params));

    tp_control::JointLimitTask::Params limit_params;
    limit_params.priority = tp_priority_joint_limits_;
    limit_params.q_min = tp_control::Vec::Constant(9, -1.0e9);
    limit_params.q_max = tp_control::Vec::Constant(9, 1.0e9);
    limit_params.margin = tp_control::Vec::Zero(9);
    limit_params.d_act = tp_joint_limits_d_act_;
    limit_params.k = tp_joint_limits_k_;
    for (int i = 0; i < 6; ++i)
    {
        const int idx = i + 3; // TP order: [base(3), arm(6)]
        limit_params.q_min(idx) = in_qmin_from_cfg_or_params_[i];
        limit_params.q_max(idx) = in_qmax_from_cfg_or_params_[i];
        limit_params.margin(idx) = tp_joint_limits_margin_;
    }
    tasks.push_back(std::make_unique<tp_control::JointLimitTask>(limit_params));

    if (tp_tcp_z_guard_enable_)
    {
        TcpHeightTask::Params ztask;
        ztask.priority = tp_priority_tcp_z_;
        ztask.z_min = tp_tcp_z_guard_z_min_;
        ztask.z_act = tp_tcp_z_guard_z_act_;
        ztask.k = tp_tcp_z_guard_k_;
        ztask.R_NB = collision_cfg.R_NB;
        tasks.push_back(std::make_unique<TcpHeightTask>(ztask));
    }

    if (tp_singularity_enable_)
    {
        tp_control::SingularityTask::Params singularity_params;
        singularity_params.priority = tp_priority_singularity_;
        singularity_params.method = tp_singularity_method_;
        if (singularity_params.method != "yoshikawa" && singularity_params.method != "det")
        {
            RCLCPP_WARN(this->get_logger(),
                        "Unknown tp.singularity.method '%s'. Falling back to 'yoshikawa'.",
                        singularity_params.method.c_str());
            singularity_params.method = "yoshikawa";
        }
        singularity_params.mu_min = std::max(1.0e-9, tp_singularity_mu_min_);
        singularity_params.mu_max = std::max(singularity_params.mu_min, tp_singularity_mu_max_);
        singularity_params.mu_safe = std::max(singularity_params.mu_max, tp_singularity_mu_safe_);
        singularity_params.k = tp_singularity_k_;
        singularity_params.fd_eps = tp_singularity_fd_eps_;
        tasks.push_back(std::make_unique<tp_control::SingularityTask>(singularity_params));
    }

    if (tp_collision_enable_)
    {
        tp_control::PointCloudCollisionTask::Params collision_params;
        collision_params.priority = tp_priority_collision_;
        collision_params.d_safe = tp_collision_d_safe_;
        collision_params.d_act = tp_collision_d_act_;
        collision_params.k = tp_collision_k_;
        collision_params.max_constraints = (tp_collision_max_constraints_ <= 0) ? 2048 : tp_collision_max_constraints_;
        collision_params.use_closest_obstacle = tp_collision_use_closest_obstacle_;
        const auto robot_point_ids = buildCollisionPointIds(collision_cfg);
        for (const int id : robot_point_ids)
        {
            collision_params.robot_points.push_back(tp_control::PointRequest{static_cast<std::int32_t>(id)});
        }
        tasks.push_back(std::make_unique<tp_control::PointCloudCollisionTask>(collision_params));
    }

    tp_control::TaskPriorityController::Params ctrl_params;
    ctrl_params.n_dof = 9;
    ctrl_params.solver.lambda = tp_solver_lambda_;
    if (tp_solver_pinv_method_ == "ldlt")
    {
        ctrl_params.solver.pinv_method = tp_control::PinvMethod::LDLT;
    }
    else
    {
        ctrl_params.solver.pinv_method = tp_control::PinvMethod::SVD;
    }

    tp_controller_ = std::make_unique<tp_control::TaskPriorityController>(
        ctrl_params, std::move(kin_provider), std::move(tasks));

    RCLCPP_INFO(this->get_logger(),
                "Task-priority controller initialized [jacobian: %s].",
                tp_use_vendor_jacobian ? "vendor WholeBodyJacobian" : "wb_jac ROS2");
}

void WbqpControllerNode::solve_tp(double x_opt[9])
{
    if (!tp_controller_)
    {
        throw std::runtime_error("TP method selected but controller is not initialized");
    }

    tp_control::RobotState state;
    state.q.resize(9);
    state.dq.resize(9);

    // TP order: [base(3), arm(6)].
    state.q(0) = x_base_;
    state.q(1) = y_base_;
    state.q(2) = theta_W2N_[2];
    state.dq(0) = dotq_prev_[6];
    state.dq(1) = dotq_prev_[7];
    state.dq(2) = dotq_prev_[8];
    for (int i = 0; i < 6; ++i)
    {
        state.q(i + 3) = q_pos_[i];
        state.dq(i + 3) = q_vel_[i];
    }

    tp_control::CommandInputs cmd;
    if (tp_tracking_mode_ == TpTrackingMode::POSE)
    {
        cmd.mode = tp_control::CommandInputs::Mode::Pose;

        const tp_control::Pose6D current_pose = computeCurrentEePoseTp(state);
        if (!tp_pose_target_initialized_)
        {
            tp_pose_goal_target_ = current_pose;
            tp_pose_target_initialized_ = true;
        }

        if (tp_pose_sub_mode_ == TpPoseSubMode::INPUT)
        {
            std::lock_guard<std::mutex> lock(tp_pose_goal_mtx_);
            tp_pose_goal_target_ = tp_pose_goal_input_;
        }
        else
        {
            const Eigen::Vector3d v_linear(u_star_[0], u_star_[1], u_star_[2]);
            const Eigen::Vector3d v_angular(u_star_[3], u_star_[4], u_star_[5]);
            tp_pose_goal_target_.p += v_linear * dt_;
            tp_pose_goal_target_.R = integrateRotationExpMap(tp_pose_goal_target_.R, v_angular, dt_);
        }

        cmd.ee_pose_target = tp_pose_goal_target_;
    }
    else
    {
        cmd.mode = tp_control::CommandInputs::Mode::Speed;
        for (int i = 0; i < 6; ++i)
        {
            cmd.ee_twist_cmd.v(i) = u_star_[i];
        }
    }

    tp_control::Output out;

    if (tp_collision_enable_)
    {
        std::vector<Eigen::Vector3d> obs;
        {
            std::lock_guard<std::mutex> lock(obstacle_points_mtx_);
            obs = obstacle_points_world_;
        }
        tp_controller_->setObstacles(obs);
    }

    tp_controller_->step(state, cmd, dt_, out);

    if (out.dq_cmd.size() != 9)
    {
        throw std::runtime_error("TP solver produced invalid command size");
    }

    // Convert TP order [base(3), arm(6)] to legacy order [arm(6), base(3)].
    for (int i = 0; i < 6; ++i)
    {
        x_opt[i] = out.dq_cmd(i + 3);
    }
    x_opt[6] = out.dq_cmd(0);
    x_opt[7] = out.dq_cmd(1);
    x_opt[8] = out.dq_cmd(2);
}

tp_control::Pose6D WbqpControllerNode::computeCurrentEePoseTp(const tp_control::RobotState& state) const
{
    const Eigen::Matrix<double, 6, 1> q_arm = state.q.segment<6>(3);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    if (!use_jac_ros2_)
    {
        T = kin_.forwardKinematicsNC(q_arm);
    }
    else if (jac_to_world_)
    {
        MobileManipulatorKinematics::BasePose2D base_pose;
        base_pose.px = state.q(0);
        base_pose.py = state.q(1);
        base_pose.yaw = state.q(2);
        T = kin_.forwardKinematicsWC(q_arm, base_pose);
    }
    else
    {
        T = kin_.forwardKinematicsNC(q_arm);
    }

    tp_control::Pose6D out;
    out.p = T.block<3,1>(0,3);
    out.R = T.block<3,3>(0,0);
    return out;
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
    const bool emergency_stop = emergency_stop_active_.load();
    std_msgs::msg::Bool state_msg;
    state_msg.data = emergency_stop;
    pub_emergency_state_->publish(state_msg);

    geometry_msgs::msg::Twist base;
    base.linear.x  = emergency_stop ? 0.0 : x_opt[6];
    base.linear.y  = emergency_stop ? 0.0 : x_opt[7];
    base.angular.z = emergency_stop ? 0.0 : x_opt[8];
    pub_cmd_vel_->publish(base);

    // When you want to publish:
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.velocity.resize(6);
    for (int i = 0; i < 6; ++i) {js.velocity[i] = emergency_stop ? 0.0 : x_opt[i];}
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
    if (emergency_stop_active_.load()) {
        pub_arm_vel_cmd_->publish(geometry_msgs::msg::Twist{});
        return;
    }

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

    pub_arm_vel_cmd_->publish(msg);
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
