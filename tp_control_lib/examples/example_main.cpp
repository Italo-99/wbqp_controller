
#include "tp_control/controller.hpp"
#include "tp_control/tasks/tracking_task.hpp"
#include "tp_control/tasks/joint_limit_task.hpp"
#include "tp_control/tasks/singularity_task.hpp"
#include "tp_control/tasks/pointcloud_collision_task.hpp"

#include <iostream>

// Minimal kinematics stub: replace with your own implementation (Pinocchio, KDL, custom).
class DummyKin final : public tp_control::IKinematicsProvider {
public:
  explicit DummyKin(int n, int n_arm) : n_(n), n_arm_(n_arm) {}

  void update(const tp_control::Vec& q) const override { q_ = q; }

  tp_control::Pose6D eePose() const override {
    tp_control::Pose6D p;
    p.p = Eigen::Vector3d(0,0,0);
    p.R = Eigen::Matrix3d::Identity();
    return p;
  }

  void eeJacobian(Eigen::Ref<tp_control::Mat> J_out) const override {
    J_out.setZero(6, n_);
    // Fill with your real Jacobian
  }

  void armJacobian(Eigen::Ref<tp_control::Mat> J_arm_out) const override {
    J_arm_out.setZero(6, n_arm_);
    // Fill with your real arm Jacobian
  }

  void embedArmGradient(const Eigen::VectorXd& grad_arm, Eigen::Ref<tp_control::Vec> grad_full_out) const override {
    grad_full_out.setZero();
    // Example mapping: arm is last n_arm joints
    grad_full_out.tail(n_arm_) = grad_arm;
  }

  Eigen::Vector3d pointPosition(const tp_control::PointRequest& req) const override {
    (void)req;
    return Eigen::Vector3d::Zero();
  }

  void pointJacobian(const tp_control::PointRequest& req, Eigen::Ref<tp_control::Mat> Jp_out) const override {
    (void)req;
    Jp_out.setZero(3, n_);
  }

  int armDofs() const override { return n_arm_; }

private:
  int n_ = 0;
  int n_arm_ = 0;
  mutable tp_control::Vec q_;
};

int main() {
  using namespace tp_control;

  const int n = 9;      // example: base(3) + arm(6)
  const int n_arm = 6;

  auto kin = std::make_unique<DummyKin>(n, n_arm);

  // Tasks
  TrackingTask::Params tp;
  tp.priority = 1;

  JointLimitTask::Params lp;
  lp.priority = 2;
  lp.q_min = Vec::Constant(n, -3.14);
  lp.q_max = Vec::Constant(n,  3.14);
  lp.margin = Vec::Constant(n, 0.1);
  lp.k = 2.0;

  SingularityTask::Params sp;
  sp.priority = 3;
  sp.mu_min = 0.02;
  sp.mu_max = 0.05;
  sp.mu_safe = 0.06;

  PointCloudCollisionTask::Params cp;
  cp.priority = 4;
  cp.d_safe = 0.15;
  cp.d_act  = 0.30;
  cp.k = 3.0;
  cp.max_constraints = 64;
  // representative points
  cp.robot_points = { PointRequest{0}, PointRequest{1}, PointRequest{2} };

  std::vector<std::unique_ptr<ITask>> tasks;
  tasks.push_back(std::make_unique<TrackingTask>(tp));
  tasks.push_back(std::make_unique<JointLimitTask>(lp));
  tasks.push_back(std::make_unique<SingularityTask>(sp));
  tasks.push_back(std::make_unique<PointCloudCollisionTask>(cp));

  TaskPriorityController::Params ctrl_p;
  ctrl_p.n_dof = n;
  ctrl_p.solver.pinv_method = PinvMethod::SVD; // or LDLT
  ctrl_p.solver.lambda = 1e-4;

  TaskPriorityController ctrl(ctrl_p, std::move(kin), std::move(tasks));

  // obstacles
  std::vector<Eigen::Vector3d> obs;
  obs.push_back(Eigen::Vector3d(0.5, 0.0, 0.0));
  ctrl.setObstacles(obs);

  RobotState st;
  st.q = Vec::Zero(n);
  st.dq = Vec::Zero(n);

  CommandInputs cmd;
  cmd.mode = CommandInputs::Mode::Pose;
  cmd.ee_pose_target.p = Eigen::Vector3d(0.3, 0.0, 0.5);
  cmd.ee_pose_target.R = Eigen::Matrix3d::Identity();

  Output out;
  ctrl.step(st, cmd, 0.01, out);

  std::cout << "dq_cmd = " << out.dq_cmd.transpose() << std::endl;
  return 0;
}
