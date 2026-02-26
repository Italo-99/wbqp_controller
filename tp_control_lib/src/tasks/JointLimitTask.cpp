
#include "tp_control/tasks/joint_limit_task.hpp"
#include <stdexcept>
#include <algorithm>

namespace tp_control {

JointLimitTask::JointLimitTask(const Params& p) : p_(p) {}

double JointLimitTask::clamp01(double x) { return std::min(1.0, std::max(0.0, x)); }

double JointLimitTask::smoothstep(double t) {
  // Smooth ramp 0->1 on [0,1]
  t = clamp01(t);
  return t * t * (3.0 - 2.0 * t);
}

void JointLimitTask::configure(int n_dof) {
  n_ = n_dof;
  if (p_.q_min.size() != n_ || p_.q_max.size() != n_ || p_.margin.size() != n_) {
    throw std::invalid_argument("JointLimitTask: q_min/q_max/margin must be size n_dof");
  }
}

void JointLimitTask::update(const Context& cx, Eigen::Ref<Mat> J_out, Eigen::Ref<Vec> d_out, Eigen::Ref<Vec> a_out) {
  // Identity Jacobian
  J_out.setIdentity();

  d_out.setZero();
  a_out.setZero();

  // Activated repulsion near limits. This is a simple symmetric choice.
  // For joint i:
  // - if q is close to lower bound -> push positive
  // - if close to upper bound -> push negative
  for (int i = 0; i < n_; ++i) {
    const double q = cx.state.q(i);
    const double qlow = p_.q_min(i) + p_.margin(i);
    const double qhigh = p_.q_max(i) - p_.margin(i);

    // lower activation
    double al = 0.0;
    if (q < qlow + p_.d_act) {
      const double t = (qlow + p_.d_act - q) / std::max(1e-12, p_.d_act);
      al = smoothstep(t);
      d_out(i) += p_.k * al * (qlow - q); // push up
    }

    // upper activation
    double au = 0.0;
    if (q > qhigh - p_.d_act) {
      const double t = (q - (qhigh - p_.d_act)) / std::max(1e-12, p_.d_act);
      au = smoothstep(t);
      d_out(i) += p_.k * au * (qhigh - q); // push down (negative)
    }

    a_out(i) = clamp01(std::max(al, au));
  }
}

} // namespace tp_control
