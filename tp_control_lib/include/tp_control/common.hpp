
#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <cstdint>
#include <span>

namespace tp_control {

using Vec = Eigen::VectorXd;
using Mat = Eigen::MatrixXd;

struct Pose6D {
  Eigen::Vector3d p;      // position
  Eigen::Matrix3d R;      // rotation matrix
};

inline Eigen::Vector3d so3Log(const Eigen::Matrix3d& R) {
  // Minimal log map for SO(3) with numerical safeguards.
  const double cos_theta = std::min(1.0, std::max(-1.0, (R.trace() - 1.0) * 0.5));
  const double theta = std::acos(cos_theta);
  if (theta < 1e-9) {
    return Eigen::Vector3d::Zero();
  }
  Eigen::Vector3d w;
  w << R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1);
  w *= 0.5 / std::sin(theta);
  return theta * w;
}

// Pose error (6D): [pos_error; rot_error]
// rot_error uses Log(R*^T R) in body-like convention; you can adapt to your preferred convention.
inline Eigen::Matrix<double, 6, 1> poseError6D(const Pose6D& current, const Pose6D& target) {
  Eigen::Matrix<double, 6, 1> e;
  e.head<3>() = (target.p - current.p);
  Eigen::Matrix3d R_err = target.R.transpose() * current.R; // note: adjust if you want opposite
  e.tail<3>() = -so3Log(R_err);
  return e;
}

struct RobotState {
  Vec q;   // size n
  Vec dq;  // size n (optional)
};

struct Twist6D {
  Eigen::Matrix<double, 6, 1> v;
};

struct CommandInputs {
  enum class Mode : std::uint8_t { Speed = 0, Pose = 1 };
  Mode mode = Mode::Pose;

  // Speed mode:
  Twist6D ee_twist_cmd;

  // Pose mode:
  Pose6D ee_pose_target;
};

struct ObstaclePointCloud {
  // Obstacles as simple points in a fixed world frame.
  // Thread-safety for async updates is handled outside, or by internal copy-on-update in the controller wrapper.
  std::vector<Eigen::Vector3d> points;
};

struct Output {
  Vec dq_cmd; // size n
};

enum class PinvMethod : std::uint8_t {
  SVD  = 0,
  LDLT = 1
};

} // namespace tp_control
