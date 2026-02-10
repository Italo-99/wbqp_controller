#pragma once

#include <Eigen/Dense>
#include <array>
#include <cmath>

class MobileManipulatorKinematics
{
public:
    struct DHParam
    {
        double a;
        double d;
        double alpha;
    };

    struct BasePose2D
    {
        double px;
        double py;
        double yaw;
    };

    MobileManipulatorKinematics()
    {
        // Default UR5e DH (standard DH)
        dh_[0] = {  0.0,    0.1625,  M_PI / 2.0 };
        dh_[1] = { -0.425,  0.0,     0.0        };
        dh_[2] = { -0.3922, 0.0,     0.0        };
        dh_[3] = {  0.0,    0.1333,  M_PI / 2.0 };
        dh_[4] = {  0.0,    0.0997, -M_PI / 2.0 };
        dh_[5] = {  0.0,    0.0996,  0.0        };

        T_NB_.setIdentity();
        T_EC_.setIdentity();
    }

    void setDH(const std::array<DHParam, 6>& dh)
    {
        dh_ = dh;
    }

    void setFixedTransformNB(const Eigen::Matrix3d& R_NB, const Eigen::Vector3d& p_NB)
    {
        T_NB_.setIdentity();
        T_NB_.block<3,3>(0,0) = R_NB;
        T_NB_.block<3,1>(0,3) = p_NB;
    }

    // TCP offset after DH tool frame E:
    // T_EC = [ Rz(rz)*Ry(ry)*Rx(rx), t ; 0 1 ] with t=[tx,ty,tz]^T
    void setTcpOffset(double tx, double ty, double tz, double rx, double ry, double rz)
    {
        const Eigen::Matrix3d R = rotZ(rz) * rotY(ry) * rotX(rx);

        T_EC_.setIdentity();
        T_EC_.block<3,3>(0,0) = R;
        T_EC_.block<3,1>(0,3) = Eigen::Vector3d(tx, ty, tz);
    }

    // ---------- Body-frame Jacobian (NO base pose needed) ----------
    // Output: w_C^N = [v_C^N; omega_C^N]
    // Input:  rdot = [qdot(6); vx; vy; Omega_z], all base velocities are in N.
    Eigen::Matrix<double, 6, 9> jacobianBody(const Eigen::Matrix<double, 6, 1>& q) const
    {
        // Compute arm Jacobian in N at TCP, plus T_NC for p_C^N
        Eigen::Matrix4d T_NC;
        Eigen::Matrix<double, 6, 6> J_arm_N;
        computeArmJacobianBodyAtTcp(q, T_NC, J_arm_N);

        const Eigen::Vector3d p_NC = T_NC.block<3,1>(0,3);

        // Base Jacobian block in N (6x3)
        Eigen::Matrix<double, 6, 3> J_base_N;
        J_base_N.setZero();

        // vx
        J_base_N.block<3,1>(0,0) = Eigen::Vector3d::UnitX();
        // vy
        J_base_N.block<3,1>(0,1) = Eigen::Vector3d::UnitY();
        // Omega_z
        {
            const Eigen::Vector3d ez = Eigen::Vector3d::UnitZ();
            J_base_N.block<3,1>(0,2) = ez.cross(p_NC);
            J_base_N.block<3,1>(3,2) = ez;
        }

        // Assemble combined Jacobian (6x9)
        Eigen::Matrix<double, 6, 9> J;
        J.setZero();
        J.block<6,6>(0,0) = J_arm_N;
        J.block<6,3>(0,6) = J_base_N;

        return J;
    }

    // ---------- World-frame Jacobian (still needs base pose) ----------
    // Output: w_C^W
    Eigen::Matrix<double, 6, 9> jacobianWorld(const Eigen::Matrix<double, 6, 1>& q, const BasePose2D& basePose) const
    {
        // Compute body Jacobian first (independent of world)
        const Eigen::Matrix<double, 6, 9> J_N = jacobianBody(q);

        // Transform twist from N to W: w^W = Ad_{T_WN} w^N
        const Eigen::Matrix4d T_WN = makeT_WN(basePose);
        const Eigen::Matrix<double, 6, 6> Ad_WN = adjoint(T_WN);

        return Ad_WN * J_N;
    }

    // FK of TCP in N (no base pose)
    Eigen::Matrix4d forwardKinematicsNC(const Eigen::Matrix<double, 6, 1>& q) const
    {
        Eigen::Matrix4d T = T_NB_;
        for (int i = 0; i < 6; i++)
        {
            T = T * dhTransform(dh_[i], q(i));
        }
        T = T * T_EC_;
        return T;
    }

    // FK of TCP in W (needs base pose)
    Eigen::Matrix4d forwardKinematicsWC(const Eigen::Matrix<double, 6, 1>& q, const BasePose2D& basePose) const
    {
        const Eigen::Matrix4d T_WN = makeT_WN(basePose);
        return T_WN * forwardKinematicsNC(q);
    }

private:
    std::array<DHParam, 6> dh_;
    Eigen::Matrix4d T_NB_;
    Eigen::Matrix4d T_EC_;

private:
    static Eigen::Matrix3d rotX(double rx)
    {
        const double c = std::cos(rx);
        const double s = std::sin(rx);

        Eigen::Matrix3d R;
        R << 1.0, 0.0, 0.0,
             0.0,   c,  -s,
             0.0,   s,   c;
        return R;
    }

    static Eigen::Matrix3d rotY(double ry)
    {
        const double c = std::cos(ry);
        const double s = std::sin(ry);

        Eigen::Matrix3d R;
        R <<   c, 0.0,   s,
             0.0, 1.0, 0.0,
              -s, 0.0,   c;
        return R;
    }

    static Eigen::Matrix3d rotZ(double rz)
    {
        const double c = std::cos(rz);
        const double s = std::sin(rz);

        Eigen::Matrix3d R;
        R <<   c,  -s, 0.0,
               s,   c, 0.0,
             0.0, 0.0, 1.0;
        return R;
    }

    static Eigen::Matrix4d makeT_WN(const BasePose2D& pose)
    {
        const double c = std::cos(pose.yaw);
        const double s = std::sin(pose.yaw);

        Eigen::Matrix4d T;
        T.setIdentity();

        T(0,0) =  c;  T(0,1) = -s;
        T(1,0) =  s;  T(1,1) =  c;

        T(0,3) = pose.px;
        T(1,3) = pose.py;
        T(2,3) = 0.0;

        return T;
    }

    static Eigen::Matrix4d dhTransform(const DHParam& dh, double q)
    {
        const double cq = std::cos(q);
        const double sq = std::sin(q);
        const double ca = std::cos(dh.alpha);
        const double sa = std::sin(dh.alpha);

        Eigen::Matrix4d T;
        T.setIdentity();

        T(0,0) = cq;        T(0,1) = -sq * ca;   T(0,2) =  sq * sa;   T(0,3) = dh.a * cq;
        T(1,0) = sq;        T(1,1) =  cq * ca;   T(1,2) = -cq * sa;   T(1,3) = dh.a * sq;
        T(2,0) = 0.0;       T(2,1) =  sa;        T(2,2) =  ca;        T(2,3) = dh.d;
        T(3,0) = 0.0;       T(3,1) = 0.0;        T(3,2) = 0.0;        T(3,3) = 1.0;

        return T;
    }

    static Eigen::Matrix3d skew(const Eigen::Vector3d& v)
    {
        Eigen::Matrix3d S;
        S <<   0.0, -v.z(),  v.y(),
             v.z(),   0.0, -v.x(),
            -v.y(),  v.x(),  0.0;
        return S;
    }

    // Ad_T maps twists: w^A = Ad_{T_AB} w^B
    static Eigen::Matrix<double, 6, 6> adjoint(const Eigen::Matrix4d& T)
    {
        const Eigen::Matrix3d R = T.block<3,3>(0,0);
        const Eigen::Vector3d p = T.block<3,1>(0,3);

        Eigen::Matrix<double, 6, 6> Ad;
        Ad.setZero();
        Ad.block<3,3>(0,0) = R;
        Ad.block<3,3>(3,3) = R;
        Ad.block<3,3>(0,3) = skew(p) * R;
        return Ad;
    }

    // Compute arm Jacobian in N directly at TCP (includes TCP offset).
    void computeArmJacobianBodyAtTcp(
        const Eigen::Matrix<double, 6, 1>& q,
        Eigen::Matrix4d& T_NC,
        Eigen::Matrix<double, 6, 6>& J_arm_N) const
    {
        // We propagate transforms in N:
        // T_NB fixed, then DH chain to E, then T_EC to TCP
        std::array<Eigen::Vector3d, 7> p_N; // origins of frames 0..6 (0=B, 6=E)
        std::array<Eigen::Vector3d, 6> z_N; // z axes of frames 0..5

        Eigen::Matrix4d T = T_NB_;

        // Frame 0 is B expressed in N
        p_N[0] = T.block<3,1>(0,3);
        z_N[0] = T.block<3,3>(0,0) * Eigen::Vector3d::UnitZ();

        for (int i = 0; i < 6; i++)
        {
            T = T * dhTransform(dh_[i], q(i));
            p_N[i + 1] = T.block<3,1>(0,3);

            if (i < 5)
            {
                z_N[i + 1] = T.block<3,3>(0,0) * Eigen::Vector3d::UnitZ();
            }
        }

        const Eigen::Matrix4d T_NE = T;
        T_NC = T_NE * T_EC_;

        const Eigen::Vector3d pC = T_NC.block<3,1>(0,3);

        J_arm_N.setZero();
        for (int i = 0; i < 6; i++)
        {
            const Eigen::Vector3d pi = p_N[i]; // origin of frame (i) = (i-1) in 1-indexed
            const Eigen::Vector3d zi = z_N[i];

            const Eigen::Vector3d Jv = zi.cross(pC - pi);
            const Eigen::Vector3d Jw = zi;

            J_arm_N.block<3,1>(0,i) = Jv;
            J_arm_N.block<3,1>(3,i) = Jw;
        }
    }
};
