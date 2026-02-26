
#include "tp_control/solver.hpp"

#include <stdexcept>
#include <algorithm>

namespace tp_control {

StackOfTasksSolver::StackOfTasksSolver(int n_dof, const Params& p)
: n_(n_dof), p_(p)
{
  if (n_ <= 0) {
    throw std::invalid_argument("StackOfTasksSolver: n_dof must be > 0");
  }
  I_.setIdentity(n_, n_);
  Q_.setIdentity(n_, n_);
  dq_.setZero(n_);

  // start with small capacity
  ensureTaskDim(6);
}

void StackOfTasksSolver::ensureTaskDim(int m) {
  if (m <= 0) {
    throw std::invalid_argument("StackOfTasksSolver: task dimension must be > 0");
  }
  if (X_.rows() == m && X_.cols() == n_) {
    return;
  }
  X_.resize(m, n_);
  Xw_.resize(m, n_);
  r_.resize(m);
  rw_.resize(m);
  sqrt_a_.resize(m);
  pinv_.resize(n_, m);
  P_.resize(n_, n_);

  X_.setZero();
  Xw_.setZero();
  r_.setZero();
  rw_.setZero();
  sqrt_a_.setZero();
  pinv_.setZero();
  P_.setZero();
}

void StackOfTasksSolver::pinvSVD(const Mat& Xw, Eigen::Ref<Mat> pinv_out) {
  // Damped SVD pseudo-inverse: X# = V * diag(s/(s^2+lambda^2)) * U^T
  Eigen::JacobiSVD<Mat> svd(Xw, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto& U = svd.matrixU();
  const auto& V = svd.matrixV();
  const auto& s = svd.singularValues();

  Mat Sd = Mat::Zero(V.cols(), U.cols());
  const double lam2 = p_.lambda * p_.lambda;

  const int r = static_cast<int>(s.size());
  for (int i = 0; i < r; ++i) {
    const double si = s(i);
    Sd(i,i) = (si) / (si*si + lam2);
  }

  pinv_out.noalias() = V * Sd * U.transpose(); // (n x m)
}

void StackOfTasksSolver::pinvLDLT(const Mat& Xw, Eigen::Ref<Mat> pinv_out) {
  // X# = (X^T X + lambda^2 I)^-1 X^T
  Mat XtX = Xw.transpose() * Xw;
  XtX.diagonal().array() += p_.lambda * p_.lambda;

  Eigen::LDLT<Mat> ldlt(XtX);
  pinv_out.noalias() = ldlt.solve(Xw.transpose());
}

void StackOfTasksSolver::computePinv(const Mat& Xw) {
  if (p_.pinv_method == PinvMethod::SVD) {
    pinvSVD(Xw, pinv_);
  } else {
    pinvLDLT(Xw, pinv_);
  }
}

void StackOfTasksSolver::solve(const std::vector<TaskBuffers>& tasks, Eigen::Ref<Vec> dq_out) {
  // Reset
  dq_.setZero();
  Q_.setIdentity();

  for (const auto& t : tasks) {
    const int m = static_cast<int>(t.d.size());
    ensureTaskDim(m);

    // X = J * Q
    X_.noalias() = t.J * Q_;

    // r = d - J*dq
    r_.noalias() = t.d - t.J * dq_;

    // row scaling by sqrt(alpha)
    // sqrt_a = sqrt(alpha)
    sqrt_a_ = t.a.array().sqrt().matrix();

    // Xw = diag(sqrt_a) * X, rw = diag(sqrt_a) * r
    Xw_ = X_;
    for (int i = 0; i < m; ++i) {
      const double w = sqrt_a_(i);
      Xw_.row(i) *= w;
      rw_(i) = w * r_(i);
    }

    // If everything deactivated, skip.
    if (sqrt_a_.maxCoeff() <= 0.0) {
      continue;
    }

    // pinv = pinv(Xw)
    computePinv(Xw_);

    // dq = dq + Q * pinv * rw
    dq_.noalias() += Q_ * (pinv_ * rw_);

    // Q = Q * (I - pinv * X)    (X is unweighted JQ)
    // P = pinv * X
    P_.noalias() = pinv_ * X_;
    Q_.noalias() = Q_ * (I_ - P_);
  }

  dq_out = dq_;
}

} // namespace tp_control
