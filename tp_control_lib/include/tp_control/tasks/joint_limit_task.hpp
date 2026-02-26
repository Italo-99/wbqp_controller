
#pragma once
#include "tp_control/task.hpp"

namespace tp_control {

class JointLimitTask final : public ITask {
public:
  struct Params {
    int priority = 2;
    Vec q_min;     // size n
    Vec q_max;     // size n
    Vec margin;    // size n (>=0)
    double k = 1.0;
    double d_act = 0.2; // activation width in joint space (radians)
    bool arm_only = false; // if true, base activations are set to 0 via user q_min/q_max choice or embedding
  };

  explicit JointLimitTask(const Params& p);

  const char* name() const override { return "JointLimitTask"; }
  int priority() const override { return p_.priority; }
  int dim() const override { return n_; }

  void configure(int n_dof) override;

  void update(const Context& cx, Eigen::Ref<Mat> J_out, Eigen::Ref<Vec> d_out, Eigen::Ref<Vec> a_out) override;

private:
  Params p_;
  int n_ = 0;

  static double smoothstep(double t);
  static double clamp01(double x);
};

} // namespace tp_control
