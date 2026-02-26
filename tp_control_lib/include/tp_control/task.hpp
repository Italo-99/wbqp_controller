
#pragma once

#include "tp_control/common.hpp"
#include "tp_control/kinematics.hpp"

namespace tp_control {

// Each task provides: J (m x n), d (m), alpha (m). Alpha is task-space activation in [0,1].
class ITask {
public:
  virtual ~ITask() = default;

  virtual const char* name() const = 0;
  virtual int priority() const = 0;     // lower number = higher priority
  virtual int dim() const = 0;          // m

  // Called once at init.
  virtual void configure(int n_dof) = 0;

  struct Context {
    const RobotState& state;
    const CommandInputs& cmd;
    const ObstaclePointCloud& obs;
    const IKinematicsProvider& kin;
    double dt;
  };

  // Fill buffers (no allocations).
  virtual void update(const Context& cx,
                      Eigen::Ref<Mat> J_out,   // m x n
                      Eigen::Ref<Vec> d_out,   // m
                      Eigen::Ref<Vec> a_out) = 0; // m
};

} // namespace tp_control
