
#pragma once
#include "tp_control/task.hpp"

namespace tp_control {

class TrackingTask final : public ITask {
public:
  struct Params {
    int priority = 1;
    Eigen::Matrix<double,6,1> Kp = (Eigen::Matrix<double,6,1>() << 2,2,2, 2,2,2).finished();
    Eigen::Matrix<double,6,1> Ki = Eigen::Matrix<double,6,1>::Zero();
    Eigen::Matrix<double,6,1> v_limit = (Eigen::Matrix<double,6,1>() << 1,1,1, 1,1,1).finished(); // optional clamp
    bool anti_windup_freeze = true;
    double z_limit = 5.0; // clamp integral state norm
  };

  explicit TrackingTask(const Params& p);

  const char* name() const override { return "TrackingTask"; }
  int priority() const override { return p_.priority; }
  int dim() const override { return 6; }

  void configure(int n_dof) override;

  void update(const Context& cx, Eigen::Ref<Mat> J_out, Eigen::Ref<Vec> d_out, Eigen::Ref<Vec> a_out) override;

private:
  Params p_;
  int n_ = 0;
  Eigen::Matrix<double,6,1> z_ = Eigen::Matrix<double,6,1>::Zero();
  bool last_overridden_ = false;
};

} // namespace tp_control
