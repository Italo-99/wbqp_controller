
#include "tp_control/tasks/tracking_task.hpp"

namespace tp_control {

TrackingTask::TrackingTask(const Params& p) : p_(p) {}

void TrackingTask::configure(int n_dof) {
  n_ = n_dof;
  z_.setZero();
  last_overridden_ = false;
}

void TrackingTask::update(const Context& cx, Eigen::Ref<Mat> J_out, Eigen::Ref<Vec> d_out, Eigen::Ref<Vec> a_out) {
  // J = EE Jacobian
  cx.kin.eeJacobian(J_out);

  // Activation always 1 for tracking
  a_out.setOnes();

  Eigen::Matrix<double,6,1> vstar = Eigen::Matrix<double,6,1>::Zero();

  if (cx.cmd.mode == CommandInputs::Mode::Speed) {
    vstar = cx.cmd.ee_twist_cmd.v;
  } else {
    // Pose mode: PI on pose error
    Pose6D cur = cx.kin.eePose();
    Eigen::Matrix<double,6,1> ex = poseError6D(cur, cx.cmd.ee_pose_target);

    // Anti-windup: keep integral bounded (simple clamp)
    if (!(p_.anti_windup_freeze && last_overridden_)) {
      z_ += ex * cx.dt;
      const double zn = z_.norm();
      if (zn > p_.z_limit && zn > 1e-12) {
        z_ *= (p_.z_limit / zn);
      }
    }

    vstar = p_.Kp.cwiseProduct(ex) + p_.Ki.cwiseProduct(z_);
  }

  // Optional clamp on twist command
  for (int i = 0; i < 6; ++i) {
    const double lim = std::max(0.0, p_.v_limit(i));
    if (lim > 0.0) {
      vstar(i) = std::min(lim, std::max(-lim, vstar(i)));
    }
  }

  d_out = vstar;

  // In a full implementation, last_overridden_ would be set based on diagnostics from higher-priority tasks.
  last_overridden_ = false;
}

} // namespace tp_control
