
#include "tp_control/controller.hpp"

#include <algorithm>
#include <stdexcept>

namespace tp_control {

static bool cmpTask(const std::unique_ptr<ITask>& a, const std::unique_ptr<ITask>& b) {
  return a->priority() < b->priority();
}

void TaskPriorityController::sortByPriority(std::vector<std::unique_ptr<ITask>>& tasks) {
  std::sort(tasks.begin(), tasks.end(), cmpTask);
}

TaskPriorityController::TaskPriorityController(const Params& p,
                                               std::unique_ptr<IKinematicsProvider> kin,
                                               std::vector<std::unique_ptr<ITask>> tasks)
: p_(p),
  kin_(std::move(kin)),
  tasks_(std::move(tasks)),
  solver_(p.n_dof, p.solver)
{
  if (p_.n_dof <= 0) {
    throw std::invalid_argument("TaskPriorityController: n_dof must be > 0");
  }
  if (!kin_) {
    throw std::invalid_argument("TaskPriorityController: kinematics provider is null");
  }
  if (tasks_.empty()) {
    throw std::invalid_argument("TaskPriorityController: at least one task is required");
  }

  sortByPriority(tasks_);

  // Configure tasks and allocate per-task buffers.
  mem_.resize(tasks_.size());
  for (std::size_t i = 0; i < tasks_.size(); ++i) {
    tasks_[i]->configure(p_.n_dof);

    const int m = tasks_[i]->dim();
    mem_[i].J.resize(m, p_.n_dof);
    mem_[i].d.resize(m);
    mem_[i].a.resize(m);

    mem_[i].J.setZero();
    mem_[i].d.setZero();
    mem_[i].a.setZero();
  }

  obstacles_.points.clear();
}

ObstaclePointCloud TaskPriorityController::snapshotObstacles() const {
  std::lock_guard<std::mutex> lk(obs_mtx_);
  return obstacles_;
}

void TaskPriorityController::setObstacles(const std::vector<Eigen::Vector3d>& points) {
  std::lock_guard<std::mutex> lk(obs_mtx_);
  obstacles_.points = points;
}

void TaskPriorityController::clearObstacles() {
  std::lock_guard<std::mutex> lk(obs_mtx_);
  obstacles_.points.clear();
}

void TaskPriorityController::step(const RobotState& state,
                                  const CommandInputs& cmd,
                                  double dt,
                                  Output& out)
{
  if (state.q.size() != p_.n_dof) {
    throw std::invalid_argument("TaskPriorityController::step: state.q has wrong size");
  }
  if (dt <= 0.0) {
    throw std::invalid_argument("TaskPriorityController::step: dt must be > 0");
  }

  // Update kinematics once per cycle
  kin_->update(state.q);

  // Snapshot obstacles (cheap copy; you can change to shared_ptr if needed)
  ObstaclePointCloud obs = snapshotObstacles();

  // Update tasks
  std::vector<TaskBuffers> stack;
  stack.reserve(tasks_.size());

  for (std::size_t i = 0; i < tasks_.size(); ++i) {
    ITask::Context cx{state, cmd, obs, *kin_, dt};
    tasks_[i]->update(cx, mem_[i].J, mem_[i].d, mem_[i].a);

    // Clamp activations defensively
    mem_[i].a = mem_[i].a.cwiseMax(0.0).cwiseMin(1.0);

    stack.push_back(TaskBuffers{mem_[i].J, mem_[i].d, mem_[i].a});
  }

  // Solve
  out.dq_cmd.resize(p_.n_dof);
  solver_.solve(stack, out.dq_cmd);

  // Optional: filtering could be applied here (not implemented in this skeleton)
}

} // namespace tp_control
