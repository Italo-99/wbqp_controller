
#pragma once

#include "tp_control/common.hpp"
#include "tp_control/task.hpp"
#include "tp_control/solver.hpp"

#include <mutex>

namespace tp_control {

class TaskPriorityController {
public:
  struct Params {
    int n_dof = 0;
    StackOfTasksSolver::Params solver;
    bool enable_filter = false; // placeholder for kino-dynamic filtering
  };

  TaskPriorityController(const Params& p,
                         std::unique_ptr<IKinematicsProvider> kin,
                         std::vector<std::unique_ptr<ITask>> tasks);

  // Asynchronous obstacle update
  void setObstacles(const std::vector<Eigen::Vector3d>& points);
  void clearObstacles();

  void step(const RobotState& state,
            const CommandInputs& cmd,
            double dt,
            Output& out);

  const std::vector<std::unique_ptr<ITask>>& tasks() const { return tasks_; }

private:
  Params p_;
  std::unique_ptr<IKinematicsProvider> kin_;
  std::vector<std::unique_ptr<ITask>> tasks_;
  StackOfTasksSolver solver_;

  // Obstacles stored internally (copy-on-update); lock is only taken on update or snapshot.
  mutable std::mutex obs_mtx_;
  ObstaclePointCloud obstacles_;

  // Per-task buffers (owned by controller for cache locality)
  struct PerTaskMem {
    Mat J;
    Vec d;
    Vec a;
  };
  std::vector<PerTaskMem> mem_;

  // helper
  static void sortByPriority(std::vector<std::unique_ptr<ITask>>& tasks);
  ObstaclePointCloud snapshotObstacles() const;
};

} // namespace tp_control
