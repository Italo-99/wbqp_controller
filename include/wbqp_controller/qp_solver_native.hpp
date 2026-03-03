#pragma once

#include <array>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Sparse>

#include <OsqpEigen/OsqpEigen.h>

namespace wbqp
{
    // All static weights/limits as global-style config (tweak here, no per-cycle cost)
    struct NativeQpConfig
    {
        // Task weights (diagonal) for ||Wt (J z - u*)||^2
        static constexpr double W_lin = 1.0;   // x,y,z
        static constexpr double W_ang = 1.0;   // rx,ry,rz

        // Regularization per block + tiny nu on all vars for SPD
        static constexpr double beta_arm  = 0.001; // joints dq1..dq6
        static constexpr double alpha_xy  = 0.002; // Vx, Vy
        static constexpr double alpha_yaw = 0.002; // Omegaz
        static constexpr double nu        = 1e-3;  // global ridge

        // Velocity limits
        static constexpr double max_dotq   = 1.0; // joints
        static constexpr double max_V      = 0.2; // per‑axis box for Vx,Vy
        static constexpr double max_Omegaz = 0.2;

        // Acceleration limits (rate on decision vars) — enforced as linear ineq
        static constexpr double qddot_max = 2.0; // joints
        static constexpr double a_lin_max = 1.0; // Vx,Vy
        static constexpr double alpha_max = 1.0; // Omegaz
    };

    // Small POD for inputs (kept flat so your node doesn’t need to build big structs)
    struct NativeQpInput
    {
        // Column-major J(6x9) flattened (like MATLAB): idx = r + 6*j
        std::array<double,54> J6x9_colmajor{};  // required
        std::array<double,6>  u_star{};         // desired ee twist in WORLD [vx vy vz wx wy wz]
        std::array<double,6>  q{};              // current joints
        std::array<double,9>  dotq_prev{};      // last cycle solution [dq(6), Vx, Vy, Omegaz]
        double                dt{0.002};
        std::array<double,6>  qmin{ {-0.0000, -2.0944, -2.3562, -2.3562, -0.0000, -3.1416} };
        std::array<double,6>  qmax{ {+1.5708, -1.4835, +0.0000, +1.5708, +1.9199, +3.1416} };

        // Optional extra linear hard constraints:
        // For each row k: extra_l[k] <= extra_A[k] * z <= extra_u[k]
        // where z = [dq1..dq6, Vx, Vy, Omegaz].
        std::vector<std::array<double,9>> extra_A_rows{};
        std::vector<double> extra_l{};
        std::vector<double> extra_u{};
    };

    // Output z* = [dq1..dq6, Vx, Vy, Omegaz]
    using NativeQpOutput = std::array<double,9>;

    class NativeQpSolver
    {
        public:
        // Main entry: returns optimal z; throws only on catastrophic OSQP setup errors.
        static NativeQpOutput solve(const NativeQpInput &in);

        private:
        // Helpers kept private: they allocate small, fixed-size Eigen stacks
        static Eigen::Matrix<double,6,9> mapJ(const std::array<double,54>& Jc);

        static void buildTask(const Eigen::Matrix<double,6,9>& J,
                            const Eigen::Matrix<double,6,1>& u,
                            const Eigen::Matrix<double,9,1>& dqprev,
                            Eigen::Matrix<double,9,9>& H,
                            Eigen::Matrix<double,9,1>& f);

        static void buildReg(Eigen::Matrix<double,9,9>& H);

        static void buildVarBounds(const NativeQpInput& in,
                                Eigen::Matrix<double,9,1> &lb,
                                Eigen::Matrix<double,9,1> &ub);

        static void buildAccelConstraints(const NativeQpInput& in,
                                        Eigen::SparseMatrix<double>& A,
                                        Eigen::VectorXd &lA, Eigen::VectorXd &uA);

        static bool solveOsqp(const Eigen::SparseMatrix<double>& Hs,
                            const Eigen::Matrix<double,9,1>& f,
                            const Eigen::SparseMatrix<double>& Aall,
                            const Eigen::VectorXd& lAll,
                            const Eigen::VectorXd& uAll,
                            const Eigen::Matrix<double,9,1>& xwarm_in,
                            NativeQpOutput &out);

        static void solveFallback(const Eigen::Matrix<double,9,9>& H,
                                const Eigen::Matrix<double,9,1>& f,
                                const Eigen::Matrix<double,9,1>& lb,
                                const Eigen::Matrix<double,9,1>& ub,
                                NativeQpOutput &out);
    };
}
