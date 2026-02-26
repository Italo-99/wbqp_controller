
#pragma once

#include "tp_control/common.hpp"

namespace tp_control {

struct TaskBuffers {
  // References to preallocated buffers owned elsewhere (controller or tasks).
  Eigen::Ref<const Mat> J;   // m x n
  Eigen::Ref<const Vec> d;   // m
  Eigen::Ref<const Vec> a;   // m
};

class StackOfTasksSolver {
public:
  struct Params {
    PinvMethod pinv_method = PinvMethod::SVD;
    double lambda = 1e-4; // damping
  };

  StackOfTasksSolver(int n_dof, const Params& p);

  void solve(const std::vector<TaskBuffers>& tasks, Eigen::Ref<Vec> dq_out);

private:
  int n_;
  Params p_;

  // Workspace
  Mat Q_;     // n x n
  Mat I_;     // n x n
  Vec dq_;    // n

  // Scratch updated per task with varying m
  Mat X_;     // m x n
  Mat Xw_;    // m x n
  Vec r_;     // m
  Vec rw_;    // m
  Vec sqrt_a_; // m
  Mat pinv_;  // n x m
  Mat P_;     // n x n (for (pinv*X) projected through Q update)

  void ensureTaskDim(int m);

  // Compute pinv_ = pinv(Xw) where Xw is m x n; result n x m
  void computePinv(const Mat& Xw);

  // helper: compute damped SVD pseudo-inverse
  void pinvSVD(const Mat& Xw, Eigen::Ref<Mat> pinv_out);

  // helper: compute damped normal-equation pseudo-inverse using LDLT
  void pinvLDLT(const Mat& Xw, Eigen::Ref<Mat> pinv_out);
};

} // namespace tp_control
