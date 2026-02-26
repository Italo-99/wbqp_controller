
#pragma once

#include "tp_control/common.hpp"

namespace tp_control {

// Request for a robot representative point (link/joint/base marker).
struct PointRequest {
  // User-defined ID to map to a point on the robot.
  // Example: base corner indices, joint indices, link sample points, etc.
  std::int32_t id = -1;
};

// Provides kinematics for the controller. You implement this using your preferred backend.
class IKinematicsProvider {
public:
  virtual ~IKinematicsProvider() = default;

  // Called once per cycle.
  virtual void update(const Vec& q) = 0;

  // EE pose and Jacobian for the full generalized coordinates (base+arm).
  virtual Pose6D eePose() const = 0;
  virtual void eeJacobian(Eigen::Ref<Mat> J_out /*6 x n*/) const = 0;

  // Arm-only Jacobian (for manipulability). Size 6 x n_arm.
  virtual void armJacobian(Eigen::Ref<Mat> J_arm_out /*6 x n_arm*/) const = 0;

  // Map arm gradient (n_arm) into full gradient (n) by placing into the arm block.
  virtual void embedArmGradient(const Eigen::VectorXd& grad_arm, Eigen::Ref<Vec> grad_full_out /*n*/) const = 0;

  // Robot representative points and their Jacobians:
  // Return current position of a requested point and fill its Jacobian (3 x n) such that p_dot = Jp * dq.
  virtual Eigen::Vector3d pointPosition(const PointRequest& req) const = 0;
  virtual void pointJacobian(const PointRequest& req, Eigen::Ref<Mat> Jp_out /*3 x n*/) const = 0;

  // Arm DOF count.
  virtual int armDofs() const = 0;
};

} // namespace tp_control
