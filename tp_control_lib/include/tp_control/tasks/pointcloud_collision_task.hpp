
#pragma once
#include "tp_control/task.hpp"
#include <deque>

namespace tp_control {

// Collision constraints built from robot representative points vs obstacle points.
// This task outputs a VARIABLE number of rows (active constraints). For SoT we keep a maximum capacity.
class PointCloudCollisionTask final : public ITask {
public:
  struct Params {
    int priority = 4;
    double d_safe = 0.15;
    double d_act = 0.30;
    double k = 2.0;
    int max_constraints = 128; // max active pairs per cycle (cap for speed)
    bool use_closest_obstacle = true; // true => one nearest obstacle per robot point
    std::vector<PointRequest> robot_points; // representative points
  };

  explicit PointCloudCollisionTask(const Params& p);

  const char* name() const override { return "PointCloudCollisionTask"; }
  int priority() const override { return p_.priority; }
  int dim() const override { return m_cap_; }

  void configure(int n_dof) override;

  void update(const Context& cx, Eigen::Ref<Mat> J_out, Eigen::Ref<Vec> d_out, Eigen::Ref<Vec> a_out) override;

  int activeConstraints() const { return m_active_; }

private:
  Params p_;
  int n_ = 0;
  int m_cap_ = 0;
  int m_active_ = 0;

  // scratch per-row
  Mat Jp_; // 3 x n
  static double clamp01(double x);
  static double smoothstep(double t);
};

} // namespace tp_control
