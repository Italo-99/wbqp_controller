
#include "tp_control/tasks/singularity_task.hpp"
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <cctype>

namespace tp_control {

SingularityTask::SingularityTask(const Params& p) : p_(p) {}

double SingularityTask::clamp01(double x) { return std::min(1.0, std::max(0.0, x)); }

double SingularityTask::smoothstep(double t) {
  t = clamp01(t);
  return t * t * (3.0 - 2.0 * t);
}

double SingularityTask::manipulabilityYoshikawa(const Mat& J) {
  // mu = sqrt(det(J J^T)). For 6x6 this equals |det(J)|.
  // Use LLT on JJ^T if SPD; otherwise clamp to 0.
  Mat M = J * J.transpose();
  Eigen::LLT<Mat> llt(M);
  if (llt.info() != Eigen::Success) {
    return 0.0;
  }
  const Mat& L = llt.matrixL();
  // det(M) = (prod diag(L))^2
  double prod = 1.0;
  for (int i = 0; i < L.rows(); ++i) {
    prod *= std::max(0.0, L(i,i));
  }
  return prod; // sqrt(det(M)) = prod(diag(L))
}

double SingularityTask::manipulabilityDet(const Mat& J) {
  if (J.rows() != J.cols()) {
    return 0.0;
  }
  return std::abs(J.determinant());
}

double SingularityTask::manipulability(const Mat& J) const {
  if (method_ == Method::DET) {
    return manipulabilityDet(J);
  }
  return manipulabilityYoshikawa(J);
}

void SingularityTask::configure(int n_dof) {
  n_ = n_dof;
  grad_full_.resize(n_);
  grad_full_.setZero();

  std::string method = p_.method;
  std::transform(method.begin(), method.end(), method.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (method == "det") {
    method_ = Method::DET;
  } else {
    method_ = Method::YOSHIKAWA;
  }

  // Will resize after we know arm dofs at runtime, but expected stable.
}

void SingularityTask::update(const Context& cx, Eigen::Ref<Mat> J_out, Eigen::Ref<Vec> d_out, Eigen::Ref<Vec> a_out) {
  // 1D task: J_out is 1 x n, d_out is 1, a_out is 1
  if (J_out.rows() != 1 || J_out.cols() != n_) {
    throw std::invalid_argument("SingularityTask: J_out must be 1 x n");
  }

  n_arm_ = cx.kin.armDofs();
  if (n_arm_ <= 0) {
    J_out.setZero();
    d_out.setZero();
    a_out.setZero();
    return;
  }

  Jarm_.resize(6, n_arm_);
  cx.kin.armJacobian(Jarm_);

  const double mu = manipulability(Jarm_);

  // Activation based on mu: 1 when mu <= mu_min, 0 when mu >= mu_max
  double alpha = 0.0;
  if (mu <= p_.mu_min) {
    alpha = 1.0;
  } else if (mu >= p_.mu_max) {
    alpha = 0.0;
  } else {
    const double t = (p_.mu_max - mu) / std::max(1e-12, (p_.mu_max - p_.mu_min));
    alpha = smoothstep(t);
  }

  // Desired push: increase mu toward mu_safe
  const double dmu_star = p_.k * alpha * (mu - p_.mu_safe); // negative if mu < mu_safe

  // Finite-difference gradient wrt arm joints: grad_arm(i) = dmu/dq_i
  grad_arm_.resize(n_arm_);
  grad_arm_.setZero();

  // NOTE: This needs kinematics evaluation at q+eps; since IKinematicsProvider only exposes update(q),
  // we must do: save q, update(q+eps), compute mu, restore. That is costly but acceptable for arm-only (small n_arm).
  // You can optimize later by analytic gradients or reuse internal kinematics derivatives.
  const Vec q0 = cx.state.q;

  for (int i = 0; i < n_arm_; ++i) {
    Vec q_plus = q0;
    // This assumes the kinematics provider knows where the arm joints live inside q.
    // embedArmGradient() will map gradients, but for FD we need a way to perturb the i-th arm joint.
    // The simplest: the provider defines an internal mapping and offers a helper; for this skeleton we approximate by:
    // perturb full q at an arm index equal to (n - n_arm + i). You should replace with a proper mapping.
    const int idx_full = (n_ - n_arm_) + i;
    if (idx_full < 0 || idx_full >= n_) {
      continue;
    }
    q_plus(idx_full) += p_.fd_eps;

    cx.kin.update(q_plus);
    Mat Jarm_p(6, n_arm_);
    cx.kin.armJacobian(Jarm_p);
    const double mu_p = manipulability(Jarm_p);

    grad_arm_(i) = (mu_p - mu) / std::max(1e-12, p_.fd_eps);
  }

  // Restore kinematics to nominal q for the rest of the pipeline.
  cx.kin.update(q0);

  // Embed grad_arm into full grad
  grad_full_.setZero();
  cx.kin.embedArmGradient(grad_arm_, grad_full_);

  // Output task row
  J_out.row(0) = grad_full_.transpose();
  d_out(0) = dmu_star;
  a_out(0) = 1.0; // already encoded via alpha in d; you may also put alpha here and set d to (mu-mu_safe)
  // Alternative: keep d = k*(mu-mu_safe), and a = alpha.
}

} // namespace tp_control
