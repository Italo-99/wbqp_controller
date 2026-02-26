
#include "tp_control/tasks/pointcloud_collision_task.hpp"
#include <algorithm>
#include <stdexcept>
#include <limits>
#include <cmath>

namespace tp_control {

PointCloudCollisionTask::PointCloudCollisionTask(const Params& p) : p_(p) {}

double PointCloudCollisionTask::clamp01(double x) { return std::min(1.0, std::max(0.0, x)); }

double PointCloudCollisionTask::smoothstep(double t) {
  t = clamp01(t);
  return t * t * (3.0 - 2.0 * t);
}

void PointCloudCollisionTask::configure(int n_dof) {
  n_ = n_dof;
  m_cap_ = std::max(1, p_.max_constraints);
  Jp_.resize(3, n_);
  Jp_.setZero();
}

void PointCloudCollisionTask::update(const Context& cx, Eigen::Ref<Mat> J_out, Eigen::Ref<Vec> d_out, Eigen::Ref<Vec> a_out) {
  // Buffers are m_cap x n, m_cap, m_cap
  if (J_out.rows() != m_cap_ || J_out.cols() != n_) {
    throw std::invalid_argument("PointCloudCollisionTask: J_out wrong shape");
  }
  if (d_out.size() != m_cap_ || a_out.size() != m_cap_) {
    throw std::invalid_argument("PointCloudCollisionTask: d_out/a_out wrong size");
  }

  J_out.setZero();
  d_out.setZero();
  a_out.setZero();
  m_active_ = 0;

  if (cx.obs.points.empty() || p_.robot_points.empty()) {
    return;
  }

  // For each robot point, find the closest obstacle point (simple O(N*M)).
  // You will replace this with KD-tree pruning. We still cap constraints by max_constraints.
  for (const auto& req : p_.robot_points) {
    if (m_active_ >= m_cap_) break;

    const Eigen::Vector3d pr = cx.kin.pointPosition(req);

    double best_d2 = std::numeric_limits<double>::infinity();
    Eigen::Vector3d best_po = Eigen::Vector3d::Zero();

    for (const auto& po : cx.obs.points) {
      const double d2 = (pr - po).squaredNorm();
      if (d2 < best_d2) {
        best_d2 = d2;
        best_po = po;
      }
    }

    const double d = std::sqrt(std::max(0.0, best_d2));
    if (d >= p_.d_act) {
      continue;
    }

    // activation between d_safe and d_act
    double alpha = 0.0;
    if (d <= p_.d_safe) {
      alpha = 1.0;
    } else {
      const double t = (p_.d_act - d) / std::max(1e-12, (p_.d_act - p_.d_safe));
      alpha = smoothstep(t);
    }

    // desired distance rate (spring-like): d* = k * alpha * (d_safe - d)
    const double d_star = p_.k * alpha * (p_.d_safe - d);

    // gradient: grad_q d = ((pr-po)^T / ||pr-po||) * Jp
    Eigen::Vector3d dir = pr - best_po;
    const double norm = dir.norm();
    if (norm < 1e-9) {
      continue;
    }
    dir /= norm;

    cx.kin.pointJacobian(req, Jp_); // 3 x n
    // row = dir^T * Jp  (1 x n)
    J_out.row(m_active_) = dir.transpose() * Jp_;
    d_out(m_active_) = d_star;
    a_out(m_active_) = 1.0; // alpha already inside d_star; alternatively set a_out=alpha and d=k*(d_safe-d)
    m_active_++;
  }
}

} // namespace tp_control
