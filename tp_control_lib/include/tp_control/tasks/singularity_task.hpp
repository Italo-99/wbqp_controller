
#pragma once
#include "tp_control/task.hpp"

namespace tp_control {

class SingularityTask final : public ITask {
public:
  struct Params {
    int priority = 3;
    double mu_min = 0.02;
    double mu_max = 0.05;
    double mu_safe = 0.06;
    double k = 1.0;
    double fd_eps = 1e-4; // finite-difference step
  };

  explicit SingularityTask(const Params& p);

  const char* name() const override { return "SingularityTask"; }
  int priority() const override { return p_.priority; }
  int dim() const override { return 1; }

  void configure(int n_dof) override;

  void update(const Context& cx, Eigen::Ref<Mat> J_out, Eigen::Ref<Vec> d_out, Eigen::Ref<Vec> a_out) override;

private:
  Params p_;
  int n_ = 0;
  int n_arm_ = 0;

  // scratch
  Mat Jarm_; // 6 x n_arm
  Vec grad_arm_;
  Vec grad_full_;

  static double clamp01(double x);
  static double smoothstep(double t);

  // Yoshikawa manipulability mu = sqrt(det(J J^T))
  static double manipulabilityYoshikawa(const Mat& J);
};

} // namespace tp_control
